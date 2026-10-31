package org.robosbse.algorithms.novelty;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SolutionSpace;

/**
 * Novelty search (Lehman and Stanley): an evolutionary algorithm that rewards behaving differently
 * instead of scoring well.
 *
 * <p>Every evaluation carries a behaviour descriptor ({@link org.robosbse.core.Evaluation#behaviour()},
 * e.g. the robot's final position and its positions at 25/50/75% of the episode). The <em>novelty</em>
 * of an individual is the mean Euclidean distance between its descriptor and the {@code k} nearest
 * descriptors among the current population, the offspring and an archive of past behaviours.
 * Individuals without a descriptor (e.g. invalid test scenarios) have novelty 0.
 *
 * <p>Each generation: tournament selection on the score, crossover and mutation, batch evaluation of
 * {@code offspring} children; novelty of parents and children; offspring whose novelty exceeds the
 * archive threshold enter the archive (first-in-first-out beyond {@code maxArchive}); then (mu + lambda)
 * truncation on the score keeps {@code populationSize} individuals. The threshold adapts: it grows by
 * 20% after a generation that added more than {@code addsPerGeneration} behaviours and shrinks by 5%
 * after five generations in a row without additions.
 *
 * <p>The score blends novelty and fitness through their ranks in the parent+offspring pool:
 * {@code score = (1 - w) * noveltyRank + w * fitnessRank} with both ranks in [0, 1] (1 = most novel /
 * fittest) and {@code w = fitnessWeight}. {@code w = 0} is pure novelty search; the context still tracks
 * the best fitness and every target hit, so the search can be judged by the same measures as any other.
 */
public final class NoveltySearch implements SearchAlgorithm {

    private final Params params;
    private final int populationSize;
    private final int offspringCount;
    private final int k;
    private final double initialThreshold;
    private final int addsPerGeneration;
    private final int maxArchive;
    private final double fitnessWeight;
    private final double crossoverRate;
    private final int tournamentSize;

    public NoveltySearch(Params params) {
        this.params = params;
        this.populationSize = params.getInt("populationSize", 30);
        this.offspringCount = params.getInt("offspring", populationSize);
        this.k = params.getInt("k", 10);
        this.initialThreshold = params.getDouble("archiveThreshold", 0.05);
        this.addsPerGeneration = params.getInt("addsPerGeneration", 4);
        this.maxArchive = params.getInt("maxArchive", 1000);
        this.fitnessWeight = params.getDouble("fitnessWeight", 0.0);
        this.crossoverRate = params.getDouble("crossoverRate", 0.8);
        this.tournamentSize = params.getInt("tournamentSize", 3);
        if (populationSize < 2 || offspringCount < 1) {
            throw new IllegalArgumentException("populationSize must be >= 2 and offspring >= 1");
        }
        if (k < 1) {
            throw new IllegalArgumentException("k must be >= 1");
        }
        if (fitnessWeight < 0 || fitnessWeight > 1) {
            throw new IllegalArgumentException("fitnessWeight must be in [0, 1]");
        }
    }

    /** An individual with its novelty and blended score. */
    record Scored<S>(Candidate<S> candidate, double novelty, double score) {
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        SolutionSpace<S> space = context.space();
        MutationOperator<S> mutation = space.mutation(params);
        CrossoverOperator<S> crossover = space.crossover(params);
        Rng rng = context.rng();
        List<double[]> archive = new ArrayList<>();
        double threshold = initialThreshold;
        int quietGenerations = 0;

        List<S> initial = new ArrayList<>(populationSize);
        for (int i = 0; i < populationSize; i++) {
            initial.add(space.random(rng));
        }
        List<Candidate<S>> evaluated = context.evaluateAll(initial);
        List<Scored<S>> population = score(evaluated, archive);
        int generation = 0;
        record(context, population, archive, threshold, generation);
        while (true) {
            List<S> children = new ArrayList<>(offspringCount);
            while (children.size() < offspringCount) {
                S p1 = tournament(population, rng).candidate().solution();
                S p2 = tournament(population, rng).candidate().solution();
                List<S> kids = rng.chance(crossoverRate) ? crossover.cross(p1, p2, rng) : List.of(p1, p2);
                for (S kid : kids) {
                    if (children.size() < offspringCount) {
                        children.add(mutation.mutate(kid, rng));
                    }
                }
            }
            List<Candidate<S>> offspring = context.evaluateAll(children);
            List<Candidate<S>> pool = new ArrayList<>(population.size() + offspring.size());
            for (Scored<S> s : population) {
                pool.add(s.candidate());
            }
            pool.addAll(offspring);
            List<Scored<S>> scored = score(pool, archive);

            int added = 0;
            for (int i = population.size(); i < scored.size(); i++) {
                Scored<S> s = scored.get(i);
                if (s.novelty() > threshold) {
                    archive.add(s.candidate().evaluation().behaviour());
                    added++;
                }
            }
            while (archive.size() > maxArchive) {
                archive.remove(0);
            }
            if (added > addsPerGeneration) {
                threshold *= 1.2;
                quietGenerations = 0;
            } else if (added == 0 && ++quietGenerations >= 5) {
                threshold *= 0.95;
                quietGenerations = 0;
            } else if (added > 0) {
                quietGenerations = 0;
            }

            scored.sort(Comparator.comparingDouble((Scored<S> s) -> -s.score()));
            population = new ArrayList<>(scored.subList(0, Math.min(populationSize, scored.size())));
            generation++;
            context.nextIteration();
            record(context, population, archive, threshold, generation);
        }
    }

    /** Novelty of every pool member against the pool and the archive, and the blended score. */
    <S> List<Scored<S>> score(List<Candidate<S>> pool, List<double[]> archive) {
        int n = pool.size();
        double[][] behaviours = new double[n][];
        for (int i = 0; i < n; i++) {
            behaviours[i] = pool.get(i).evaluation().behaviour();
        }
        double[] novelty = new double[n];
        for (int i = 0; i < n; i++) {
            novelty[i] = novelty(behaviours, i, archive, k);
        }
        double[] noveltyRank = ranks(novelty, true);
        double[] fitness = new double[n];
        for (int i = 0; i < n; i++) {
            fitness[i] = pool.get(i).fitness();
        }
        double[] fitnessRank = ranks(fitness, false);
        List<Scored<S>> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            double score = (1 - fitnessWeight) * noveltyRank[i] + fitnessWeight * fitnessRank[i];
            out.add(new Scored<>(pool.get(i), novelty[i], score));
        }
        return out;
    }

    /**
     * Mean distance from {@code behaviours[self]} to its {@code k} nearest neighbours among the other
     * pool members and the archive (fewer if fewer exist); 0 without a descriptor.
     */
    static double novelty(double[][] behaviours, int self, List<double[]> archive, int k) {
        double[] b = behaviours[self];
        if (b.length == 0) {
            return 0;
        }
        double[] distances = new double[behaviours.length - 1 + archive.size()];
        int count = 0;
        for (int j = 0; j < behaviours.length; j++) {
            if (j != self && behaviours[j].length == b.length) {
                distances[count++] = distance(b, behaviours[j]);
            }
        }
        for (double[] a : archive) {
            if (a.length == b.length) {
                distances[count++] = distance(b, a);
            }
        }
        if (count == 0) {
            return 0;
        }
        Arrays.sort(distances, 0, count);
        int m = Math.min(k, count);
        double sum = 0;
        for (int i = 0; i < m; i++) {
            sum += distances[i];
        }
        return sum / m;
    }

    static double distance(double[] a, double[] b) {
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            double d = a[i] - b[i];
            sum += d * d;
        }
        return Math.sqrt(sum);
    }

    /** Ranks scaled to [0, 1]; 1 for the largest value when {@code largerIsBetter}, else the smallest. */
    static double[] ranks(double[] values, boolean largerIsBetter) {
        int n = values.length;
        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) {
            order[i] = i;
        }
        Arrays.sort(order, Comparator.comparingDouble(i -> largerIsBetter ? values[i] : -values[i]));
        double[] ranks = new double[n];
        for (int r = 0; r < n; r++) {
            ranks[order[r]] = n == 1 ? 1 : (double) r / (n - 1);
        }
        return ranks;
    }

    private <S> Scored<S> tournament(List<Scored<S>> population, Rng rng) {
        Scored<S> best = null;
        for (int i = 0; i < tournamentSize; i++) {
            Scored<S> s = population.get(rng.nextInt(population.size()));
            if (best == null || s.score() > best.score()) {
                best = s;
            }
        }
        return best;
    }

    private <S> void record(SearchContext<S> context, List<Scored<S>> population, List<double[]> archive,
                            double threshold, int generation) {
        double meanNovelty = 0;
        double best = Double.POSITIVE_INFINITY;
        for (Scored<S> s : population) {
            meanNovelty += s.novelty() / population.size();
            best = Math.min(best, s.candidate().fitness());
        }
        long x = context.evaluations();
        context.series("meanNovelty", x, meanNovelty);
        context.series("archiveSize", x, archive.size());
        context.series("populationBest", x, best);
        context.stat("generation", generation);
        context.stat("archiveSize", archive.size());
        context.stat("noveltyThreshold", threshold);
        context.stat("meanNovelty", meanNovelty);
    }
}
