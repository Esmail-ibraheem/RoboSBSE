package org.robosbse.core.representation;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.Move;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SolutionSpace;

/**
 * Box-constrained real vectors, the representation shared by route optimisation (waypoint
 * coordinates), scenario generation for testing (bounded scenario variables) and — later —
 * controller tuning and inverse kinematics.
 *
 * <p>Genes may be grouped into blocks of {@code blockSize} (e.g. the x/y pair of one waypoint).
 * Crossover points respect block boundaries so that a waypoint is never split, and the random
 * neighbour perturbs one whole block. Values leaving the box are clamped to the bounds.
 *
 * <p>Operator parameters (read from the algorithm configuration):
 * <ul>
 *   <li>{@code mutation}: {@code gaussian} (default) or {@code uniform-reset}</li>
 *   <li>{@code mutationRate}: per-gene probability; non-positive means 1/n</li>
 *   <li>{@code mutationSigma}: Gaussian standard deviation as a fraction of the gene range</li>
 *   <li>{@code crossover}: {@code one-point} (default), {@code two-point}, {@code uniform},
 *       {@code arithmetic}</li>
 *   <li>{@code stepSize}: neighbourhood step as a fraction of the gene range</li>
 * </ul>
 */
public final class RealVectorSpace implements SolutionSpace<RealVector> {

    public static final List<String> MUTATIONS = List.of("gaussian", "uniform-reset");
    public static final List<String> CROSSOVERS = List.of("one-point", "two-point", "uniform", "arithmetic");

    private final double[] lower;
    private final double[] upper;
    private final List<String> names;
    private final int blockSize;

    public RealVectorSpace(double[] lower, double[] upper, List<String> names, int blockSize) {
        if (lower.length != upper.length || lower.length == 0) {
            throw new IllegalArgumentException("bounds must be non-empty and of equal length");
        }
        if (blockSize < 1 || lower.length % blockSize != 0) {
            throw new IllegalArgumentException("dimension must be a multiple of blockSize");
        }
        for (int i = 0; i < lower.length; i++) {
            if (!(lower[i] <= upper[i])) {
                throw new IllegalArgumentException("lower > upper at gene " + i);
            }
        }
        this.lower = lower.clone();
        this.upper = upper.clone();
        this.names = names == null ? defaultNames(lower.length) : List.copyOf(names);
        this.blockSize = blockSize;
    }

    public static RealVectorSpace uniform(int dimension, double lo, double hi) {
        double[] l = new double[dimension];
        double[] u = new double[dimension];
        java.util.Arrays.fill(l, lo);
        java.util.Arrays.fill(u, hi);
        return new RealVectorSpace(l, u, null, 1);
    }

    private static List<String> defaultNames(int n) {
        List<String> names = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            names.add("x" + i);
        }
        return names;
    }

    public double lower(int i) {
        return lower[i];
    }

    public double upper(int i) {
        return upper[i];
    }

    public double range(int i) {
        return upper[i] - lower[i];
    }

    public List<String> names() {
        return names;
    }

    public int blockSize() {
        return blockSize;
    }

    public double clamp(int i, double v) {
        return Math.max(lower[i], Math.min(upper[i], v));
    }

    public RealVector clamp(double[] values) {
        double[] out = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            out[i] = clamp(i, values[i]);
        }
        return RealVector.wrap(out);
    }

    @Override
    public String representation() {
        return "real-vector";
    }

    @Override
    public int dimension() {
        return lower.length;
    }

    @Override
    public RealVector random(Rng rng) {
        double[] v = new double[lower.length];
        for (int i = 0; i < v.length; i++) {
            v[i] = rng.uniform(lower[i], upper[i]);
        }
        return RealVector.wrap(v);
    }

    @Override
    public List<String> mutationOperators() {
        return MUTATIONS;
    }

    @Override
    public List<String> crossoverOperators() {
        return CROSSOVERS;
    }

    @Override
    public MutationOperator<RealVector> mutation(Params params) {
        String name = SolutionSpace.operator(params, "mutation", MUTATIONS, representation());
        double configuredRate = params.getDouble("mutationRate", -1);
        double rate = configuredRate > 0 ? configuredRate : 1.0 / dimension();
        double sigma = params.getDouble("mutationSigma", 0.1);
        return switch (name) {
            case "gaussian" -> (parent, rng) -> mutateGenes(parent, rng, rate,
                    (i, x) -> clamp(i, x + rng.gaussian() * sigma * range(i)));
            case "uniform-reset" -> (parent, rng) -> mutateGenes(parent, rng, rate,
                    (i, x) -> rng.uniform(lower[i], upper[i]));
            default -> throw new IllegalArgumentException("unknown mutation '" + name + "', expected " + MUTATIONS);
        };
    }

    private interface GeneChange {
        double apply(int gene, double value);
    }

    /**
     * Applies {@code change} to each gene with probability {@code rate}. If no gene was selected, one
     * random gene is changed so that mutation never silently returns a clone (which would waste a
     * fitness evaluation on a duplicate).
     */
    private RealVector mutateGenes(RealVector parent, Rng rng, double rate, GeneChange change) {
        double[] v = parent.toArray();
        boolean changed = false;
        for (int i = 0; i < v.length; i++) {
            if (rng.chance(rate)) {
                v[i] = change.apply(i, v[i]);
                changed = true;
            }
        }
        if (!changed) {
            int i = rng.nextInt(v.length);
            v[i] = change.apply(i, v[i]);
        }
        return RealVector.wrap(v);
    }

    @Override
    public CrossoverOperator<RealVector> crossover(Params params) {
        String name = SolutionSpace.operator(params, "crossover", CROSSOVERS, representation());
        int blocks = dimension() / blockSize;
        return switch (name) {
            case "one-point" -> (a, b, rng) -> {
                if (blocks < 2) {
                    return List.of(a, b);
                }
                int cut = (1 + rng.nextInt(blocks - 1)) * blockSize;
                return swapRange(a, b, cut, dimension());
            };
            case "two-point" -> (a, b, rng) -> {
                if (blocks < 3) {
                    return List.of(a, b);
                }
                int c1 = 1 + rng.nextInt(blocks - 1);
                int c2 = 1 + rng.nextInt(blocks - 1);
                while (c2 == c1) {
                    c2 = 1 + rng.nextInt(blocks - 1);
                }
                return swapRange(a, b, Math.min(c1, c2) * blockSize, Math.max(c1, c2) * blockSize);
            };
            case "uniform" -> (a, b, rng) -> {
                double[] x = a.toArray();
                double[] y = b.toArray();
                for (int blk = 0; blk < blocks; blk++) {
                    if (rng.chance(0.5)) {
                        for (int i = blk * blockSize; i < (blk + 1) * blockSize; i++) {
                            double t = x[i];
                            x[i] = y[i];
                            y[i] = t;
                        }
                    }
                }
                return List.of(RealVector.wrap(x), RealVector.wrap(y));
            };
            case "arithmetic" -> (a, b, rng) -> {
                double alpha = rng.nextDouble();
                double[] x = new double[dimension()];
                double[] y = new double[dimension()];
                for (int i = 0; i < x.length; i++) {
                    x[i] = alpha * a.get(i) + (1 - alpha) * b.get(i);
                    y[i] = (1 - alpha) * a.get(i) + alpha * b.get(i);
                }
                return List.of(RealVector.wrap(x), RealVector.wrap(y));
            };
            default -> throw new IllegalArgumentException("unknown crossover '" + name + "', expected " + CROSSOVERS);
        };
    }

    private static List<RealVector> swapRange(RealVector a, RealVector b, int from, int to) {
        double[] x = a.toArray();
        double[] y = b.toArray();
        for (int i = from; i < to; i++) {
            double t = x[i];
            x[i] = y[i];
            y[i] = t;
        }
        return List.of(RealVector.wrap(x), RealVector.wrap(y));
    }

    /**
     * Neighbourhood with {@code stepSize} as a fraction of each gene's range. The enumerable
     * neighbourhood moves one gene by +/- one step (at most 2n neighbours); the random neighbour
     * perturbs every gene of one randomly chosen block by a Gaussian with that standard deviation.
     */
    @Override
    public Neighbourhood<RealVector> neighbourhood(Params params) {
        double step = params.getDouble("stepSize", 0.05);
        return new Neighbourhood<>() {
            @Override
            public List<RealVector> all(RealVector s) {
                List<RealVector> result = new ArrayList<>(2 * dimension());
                for (int i = 0; i < dimension(); i++) {
                    for (int sign = -1; sign <= 1; sign += 2) {
                        double moved = clamp(i, s.get(i) + sign * step * range(i));
                        if (moved != s.get(i)) {
                            result.add(s.with(i, moved));
                        }
                    }
                }
                return result;
            }

            /** Attribute "gene i up/down"; the inverse move (same gene, other direction) becomes tabu. */
            @Override
            public List<Move<RealVector>> moves(RealVector s) {
                List<Move<RealVector>> result = new ArrayList<>(2 * dimension());
                for (int i = 0; i < dimension(); i++) {
                    for (int sign = -1; sign <= 1; sign += 2) {
                        double moved = clamp(i, s.get(i) + sign * step * range(i));
                        if (moved != s.get(i)) {
                            result.add(new Move<>(s.with(i, moved), i + (sign > 0 ? "+" : "-"),
                                    i + (sign > 0 ? "-" : "+")));
                        }
                    }
                }
                return result;
            }

            @Override
            public RealVector random(RealVector s, Rng rng) {
                double[] v = s.toArray();
                int block = rng.nextInt(dimension() / blockSize);
                for (int i = block * blockSize; i < (block + 1) * blockSize; i++) {
                    v[i] = clamp(i, v[i] + rng.gaussian() * step * range(i));
                }
                return RealVector.wrap(v);
            }
        };
    }

    /** Root-mean-square of range-normalised gene differences, in [0, 1]. */
    @Override
    public double distance(RealVector a, RealVector b) {
        double sum = 0;
        for (int i = 0; i < dimension(); i++) {
            double r = range(i);
            double d = r > 0 ? (a.get(i) - b.get(i)) / r : 0;
            sum += d * d;
        }
        return Math.sqrt(sum / dimension());
    }
}
