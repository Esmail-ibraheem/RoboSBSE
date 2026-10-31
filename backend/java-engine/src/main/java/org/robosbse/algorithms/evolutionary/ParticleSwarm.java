package org.robosbse.algorithms.evolutionary;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

/**
 * Particle swarm optimisation (Kennedy and Eberhart; constriction-equivalent defaults of Clerc and
 * Kennedy: inertia 0.7298, c1 = c2 = 1.49618).
 *
 * <p>Each particle has a position x, a velocity v and a personal best p. Per iteration and dimension
 * (with r1, r2 ~ U(0,1) drawn per dimension):
 * <pre>
 *   v = w v + c1 r1 (p - x) + c2 r2 (g - x),   |v_d| <= vMax * range_d
 *   x = x + v                                    (clamped; the velocity component is zeroed at a bound)
 * </pre>
 * where g is the global best ({@code topology = global}) or the best personal best among the particle
 * and its two ring neighbours ({@code topology = ring}). The guide of every particle is chosen at the
 * start of an iteration. Two update modes:
 * <ul>
 *   <li><b>immediate</b> (default): particles move and are evaluated one after another, and a personal
 *       best is replaced as soon as the particle improves on it. A particle whose guide already improved
 *       earlier in the same iteration is therefore attracted to the guide's new position.</li>
 *   <li><b>synchronous</b>: every particle moves with the personal bests from the start of the
 *       iteration, and the whole swarm is then evaluated as one batch
 *       ({@link SearchContext#evaluateAll}, parallel when the context has several threads).</li>
 * </ul>
 * Both modes draw the same random numbers. The inertia can decrease linearly to {@code inertiaEnd}
 * over the evaluation budget. Real-vector problems only.
 */
public final class ParticleSwarm implements SearchAlgorithm {

    private final int swarmSize;
    private final double inertia;
    private final double inertiaEnd;
    private final double cognitive;
    private final double social;
    private final double maxVelocity;
    private final boolean ring;
    private final boolean synchronous;

    public ParticleSwarm(Params params) {
        this.swarmSize = params.getInt("swarmSize", 30);
        this.inertia = params.getDouble("inertia", 0.7298);
        this.inertiaEnd = params.getDouble("inertiaEnd", -1);
        this.cognitive = params.getDouble("cognitive", 1.49618);
        this.social = params.getDouble("social", 1.49618);
        this.maxVelocity = params.getDouble("maxVelocity", 0.2);
        this.ring = params.getString("topology", "global").equals("ring");
        String update = params.getString("update", "immediate");
        if (!List.of("immediate", "synchronous").contains(update)) {
            throw new IllegalArgumentException("update must be 'immediate' or 'synchronous'");
        }
        this.synchronous = update.equals("synchronous");
        if (swarmSize < 2) {
            throw new IllegalArgumentException("swarmSize must be >= 2");
        }
    }

    @Override
    public <S> void run(SearchContext<S> raw) {
        SearchContext<RealVector> context = RealVectorSupport.realContext(raw, "PSO");
        RealVectorSpace space = (RealVectorSpace) context.space();
        Rng rng = context.rng();
        int n = space.dimension();
        double[][] x = new double[swarmSize][];
        double[][] v = new double[swarmSize][n];
        Candidate<RealVector>[] personal = newArray(swarmSize);
        List<RealVector> initial = new ArrayList<>(swarmSize);
        for (int i = 0; i < swarmSize; i++) {
            x[i] = space.random(rng).toArray();
            for (int d = 0; d < n; d++) {
                v[i][d] = rng.uniform(-maxVelocity, maxVelocity) * space.range(d) * 0.5;
            }
            initial.add(new RealVector(x[i]));
        }
        List<Candidate<RealVector>> evaluatedInitial = context.evaluateAll(initial);
        for (int i = 0; i < swarmSize; i++) {
            personal[i] = evaluatedInitial.get(i);
        }
        long totalBudget = context.budget().maxEvaluations();
        while (true) {
            double w = inertiaEnd >= 0
                    ? inertia + (inertiaEnd - inertia) * Math.min(1.0, (double) context.evaluations() / totalBudget)
                    : inertia;
            int[] guide = new int[swarmSize];
            for (int i = 0; i < swarmSize; i++) {
                guide[i] = ring ? ringBest(personal, i) : globalBest(personal);
            }
            Candidate<RealVector>[] start = synchronous ? personal.clone() : personal;
            List<RealVector> moved = new ArrayList<>(swarmSize);
            for (int i = 0; i < swarmSize; i++) {
                RealVector p = personal[i].solution();
                RealVector g = start[guide[i]].solution();
                for (int d = 0; d < n; d++) {
                    double vMax = maxVelocity * space.range(d);
                    double vel = w * v[i][d]
                            + cognitive * rng.nextDouble() * (p.get(d) - x[i][d])
                            + social * rng.nextDouble() * (g.get(d) - x[i][d]);
                    vel = Math.max(-vMax, Math.min(vMax, vel));
                    double pos = x[i][d] + vel;
                    if (pos < space.lower(d) || pos > space.upper(d)) {
                        pos = space.clamp(d, pos);
                        vel = 0;
                    }
                    v[i][d] = vel;
                    x[i][d] = pos;
                }
                if (synchronous) {
                    moved.add(new RealVector(x[i]));
                    continue;
                }
                Candidate<RealVector> c = context.evaluate(new RealVector(x[i]));
                if (c.fitness() < personal[i].fitness()) {
                    personal[i] = c;
                }
            }
            if (synchronous) {
                List<Candidate<RealVector>> evaluated = context.evaluateAll(moved);
                for (int i = 0; i < swarmSize; i++) {
                    if (evaluated.get(i).fitness() < personal[i].fitness()) {
                        personal[i] = evaluated.get(i);
                    }
                }
            }
            context.nextIteration();
            context.stat("inertia", w);
            if (context.iteration() % 5 == 0) {
                context.series("swarmDiversity", context.evaluations(), diversity(x, space));
                context.series("globalBest", context.evaluations(), personal[globalBest(personal)].fitness());
            }
        }
    }

    /** Index of the best personal best (exposed through invariants tests via the context's best). */
    static <T> int globalBest(Candidate<T>[] personal) {
        int best = 0;
        for (int i = 1; i < personal.length; i++) {
            if (personal[i].fitness() < personal[best].fitness()) {
                best = i;
            }
        }
        return best;
    }

    static <T> int ringBest(Candidate<T>[] personal, int i) {
        int n = personal.length;
        int best = i;
        for (int k : new int[] {(i + n - 1) % n, (i + 1) % n}) {
            if (personal[k].fitness() < personal[best].fitness()) {
                best = k;
            }
        }
        return best;
    }

    /** Mean normalised distance of the particles to the swarm centroid. */
    private static double diversity(double[][] x, RealVectorSpace space) {
        int n = space.dimension();
        double[] centre = new double[n];
        for (double[] p : x) {
            for (int d = 0; d < n; d++) {
                centre[d] += p[d] / x.length;
            }
        }
        double total = 0;
        for (double[] p : x) {
            double s = 0;
            for (int d = 0; d < n; d++) {
                double r = space.range(d) > 0 ? (p[d] - centre[d]) / space.range(d) : 0;
                s += r * r;
            }
            total += Math.sqrt(s / n);
        }
        return total / x.length;
    }

    @SuppressWarnings("unchecked")
    private static Candidate<RealVector>[] newArray(int n) {
        return (Candidate<RealVector>[]) new Candidate[n];
    }
}
