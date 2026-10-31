package org.robosbse.algorithms.evolutionary;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.Rng;

/** Parent selection for population-based algorithms (fitness is minimised). */
public interface Selection {

    List<String> NAMES = List.of("tournament", "roulette", "rank");

    <S> Candidate<S> select(List<Candidate<S>> population, Rng rng);

    /** Tournament selection: the best of {@code size} individuals drawn uniformly with replacement. */
    static Selection tournament(int size) {
        if (size < 1) {
            throw new IllegalArgumentException("tournament size must be >= 1");
        }
        return new Selection() {
            @Override
            public <S> Candidate<S> select(List<Candidate<S>> population, Rng rng) {
                Candidate<S> best = null;
                for (int i = 0; i < size; i++) {
                    Candidate<S> c = population.get(rng.nextInt(population.size()));
                    if (best == null || c.fitness() < best.fitness()) {
                        best = c;
                    }
                }
                return best;
            }
        };
    }

    /**
     * Fitness-proportional (roulette-wheel) selection adapted to minimisation.
     *
     * <p>Plain fitness-proportional selection is only valid for non-negative fitness that is
     * maximised. Here fitness is minimised and may be negative (e.g. robustness values in testing),
     * so the weights use <em>windowing</em>: {@code w_i = (f_worst - f_i) + eps}, where
     * {@code eps} is 1% of the population's fitness range. If all individuals have equal fitness the
     * selection is uniform.
     */
    static Selection roulette() {
        return new Selection() {
            @Override
            public <S> Candidate<S> select(List<Candidate<S>> population, Rng rng) {
                double worst = Double.NEGATIVE_INFINITY;
                double best = Double.POSITIVE_INFINITY;
                for (Candidate<S> c : population) {
                    worst = Math.max(worst, c.fitness());
                    best = Math.min(best, c.fitness());
                }
                double range = worst - best;
                if (!(range > 0) || Double.isInfinite(range)) {
                    return population.get(rng.nextInt(population.size()));
                }
                double eps = 0.01 * range;
                double total = 0;
                for (Candidate<S> c : population) {
                    total += worst - c.fitness() + eps;
                }
                double r = rng.nextDouble() * total;
                for (Candidate<S> c : population) {
                    r -= worst - c.fitness() + eps;
                    if (r <= 0) {
                        return c;
                    }
                }
                return population.get(population.size() - 1);
            }
        };
    }

    /**
     * Linear ranking selection with selection pressure {@code s} in [1, 2]: the best individual is
     * selected with probability {@code s/N}, the worst with {@code (2-s)/N}.
     */
    static Selection rank(double pressure) {
        if (pressure < 1 || pressure > 2) {
            throw new IllegalArgumentException("rank pressure must be in [1, 2]");
        }
        return new Selection() {
            @Override
            public <S> Candidate<S> select(List<Candidate<S>> population, Rng rng) {
                List<Candidate<S>> sorted = new ArrayList<>(population);
                sorted.sort(Comparator.comparingDouble(Candidate::fitness));
                int n = sorted.size();
                if (n == 1) {
                    return sorted.get(0);
                }
                double r = rng.nextDouble();
                double cumulative = 0;
                for (int i = 0; i < n; i++) {
                    int rankFromWorst = n - 1 - i;
                    double p = (2 - pressure) / n + 2.0 * rankFromWorst * (pressure - 1) / (n * (n - 1.0));
                    cumulative += p;
                    if (r < cumulative) {
                        return sorted.get(i);
                    }
                }
                return sorted.get(n - 1);
            }
        };
    }

    static Selection byName(String name, int tournamentSize, double rankPressure) {
        return switch (name) {
            case "tournament" -> tournament(tournamentSize);
            case "roulette" -> roulette();
            case "rank" -> rank(rankPressure);
            default -> throw new IllegalArgumentException("unknown selection '" + name + "', expected " + NAMES);
        };
    }
}
