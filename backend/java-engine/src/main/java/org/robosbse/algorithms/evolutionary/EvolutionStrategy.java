package org.robosbse.algorithms.evolutionary;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

/**
 * Evolution strategy (mu/rho +, lambda) with Gaussian mutation and step-size control.
 *
 * <p>Each generation creates lambda offspring: rho parents are drawn without replacement, their
 * object variables and step sizes are recombined by intermediate (mean) recombination, the step
 * size is mutated and then every coordinate is perturbed by {@code sigma * range_d * N(0,1)}.
 * Survivor selection is <b>comma</b> (best mu offspring; requires lambda >= mu) or <b>plus</b>
 * (best mu of parents and offspring, elitist).
 *
 * <p>Step-size control:
 * <ul>
 *   <li>{@code self-adaptive}: every individual carries its sigma, mutated log-normally
 *       {@code sigma' = sigma * exp(tau * N(0,1))}, tau = 1/sqrt(n) (Schwefel);</li>
 *   <li>{@code one-fifth}: one global sigma adapted every {@code adaptInterval} generations by
 *       Rechenberg's rule — increase (divide by 0.82) if more than 1/5 of the offspring improved on
 *       their best parent, decrease (multiply by 0.82) otherwise.</li>
 * </ul>
 * Real-vector problems only.
 */
public final class EvolutionStrategy implements SearchAlgorithm {

    private record Individual(Candidate<RealVector> candidate, double sigma) {
    }

    private final int mu;
    private final int lambda;
    private final int rho;
    private final boolean plus;
    private final double initialSigma;
    private final double minSigma;
    private final boolean oneFifth;
    private final int adaptInterval;

    public EvolutionStrategy(Params params) {
        this.mu = params.getInt("mu", 5);
        this.lambda = params.getInt("lambda", 35);
        this.rho = params.getInt("rho", 2);
        this.plus = params.getString("selection", "comma").equals("plus");
        this.initialSigma = params.getDouble("initialSigma", 0.1);
        this.minSigma = params.getDouble("minSigma", 1e-5);
        this.oneFifth = params.getString("stepControl", "self-adaptive").equals("one-fifth");
        this.adaptInterval = params.getInt("adaptInterval", 5);
        if (mu < 1 || lambda < 1 || rho < 1 || rho > mu) {
            throw new IllegalArgumentException("ES needs mu >= 1, lambda >= 1 and 1 <= rho <= mu");
        }
        if (!plus && lambda < mu) {
            throw new IllegalArgumentException("comma selection needs lambda >= mu");
        }
    }

    @Override
    public <S> void run(SearchContext<S> raw) {
        SearchContext<RealVector> context = RealVectorSupport.realContext(raw, "Evolution strategy");
        RealVectorSpace space = (RealVectorSpace) context.space();
        Rng rng = context.rng();
        int n = space.dimension();
        double tau = 1.0 / Math.sqrt(n);
        double globalSigma = initialSigma;
        List<RealVector> initial = new ArrayList<>();
        for (int i = 0; i < mu; i++) {
            initial.add(space.random(rng));
        }
        List<Individual> parents = new ArrayList<>();
        for (Candidate<RealVector> c : context.evaluateAll(initial)) {
            parents.add(new Individual(c, initialSigma));
        }
        int successes = 0;
        int trials = 0;
        int generation = 0;
        while (true) {
            List<RealVector> children = new ArrayList<>(lambda);
            double[] sigmas = new double[lambda];
            double[] parentBest = new double[lambda];
            for (int k = 0; k < lambda; k++) {
                List<Individual> chosen = drawParents(parents, rng);
                double[] x = new double[n];
                double sigma = 0;
                double bestParent = Double.POSITIVE_INFINITY;
                for (Individual p : chosen) {
                    for (int d = 0; d < n; d++) {
                        x[d] += p.candidate().solution().get(d) / chosen.size();
                    }
                    sigma += Math.log(p.sigma()) / chosen.size();
                    bestParent = Math.min(bestParent, p.candidate().fitness());
                }
                sigma = oneFifth ? globalSigma : Math.max(minSigma, Math.exp(sigma) * Math.exp(tau * rng.gaussian()));
                for (int d = 0; d < n; d++) {
                    x[d] = space.clamp(d, x[d] + sigma * space.range(d) * rng.gaussian());
                }
                children.add(new RealVector(x));
                sigmas[k] = sigma;
                parentBest[k] = bestParent;
            }
            List<Candidate<RealVector>> evaluated = context.evaluateAll(children);
            List<Individual> offspring = new ArrayList<>(lambda);
            for (int k = 0; k < lambda; k++) {
                Candidate<RealVector> child = evaluated.get(k);
                trials++;
                if (child.fitness() < parentBest[k]) {
                    successes++;
                }
                offspring.add(new Individual(child, sigmas[k]));
            }
            List<Individual> pool = new ArrayList<>(offspring);
            if (plus) {
                pool.addAll(parents);
            }
            pool.sort(Comparator.comparingDouble(i -> i.candidate().fitness()));
            parents = new ArrayList<>(pool.subList(0, mu));
            generation++;
            context.nextIteration();
            if (oneFifth && generation % adaptInterval == 0) {
                double rate = (double) successes / trials;
                globalSigma = Math.max(minSigma, rate > 0.2 ? globalSigma / 0.82 : rate < 0.2 ? globalSigma * 0.82 : globalSigma);
                context.stat("successRate", rate);
                successes = 0;
                trials = 0;
            }
            double meanSigma = 0;
            for (Individual p : parents) {
                meanSigma += p.sigma() / parents.size();
            }
            context.stat("generation", generation);
            context.stat("meanSigma", meanSigma);
            context.series("sigma", context.evaluations(), meanSigma);
            context.series("populationBest", context.evaluations(), parents.get(0).candidate().fitness());
        }
    }

    private List<Individual> drawParents(List<Individual> parents, Rng rng) {
        List<Individual> pool = new ArrayList<>(parents);
        rng.shuffle(pool);
        return pool.subList(0, Math.min(rho, pool.size()));
    }
}
