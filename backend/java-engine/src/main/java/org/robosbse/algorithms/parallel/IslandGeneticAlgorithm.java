package org.robosbse.algorithms.parallel;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.robosbse.algorithms.evolutionary.GeneticAlgorithm;
import org.robosbse.core.Candidate;
import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SolutionSpace;

/**
 * Island-model genetic algorithm: {@code islands} generational GAs of {@code islandSize} individuals
 * evolve separately and exchange individuals every {@code migrationInterval} generations.
 *
 * <p>At a migration, every island sends {@code migrants} individuals (its best, or random members)
 * to its successor ({@code topology = ring}) or to every other island ({@code complete}). Emigrants
 * are chosen from all islands before any island receives immigrants, so the island order does not
 * matter. Immigrants replace the receiving island's worst members, so an island's best always survives;
 * at most {@code islandSize - 1} immigrants are accepted per migration, in source-island order.
 * {@code migrationInterval = 0} disables migration: the islands are then independent GA runs that
 * share one budget.
 *
 * <p>All islands breed with the run's random stream, island by island, and the offspring of all
 * islands are evaluated as one batch ({@link SearchContext#evaluateAll}), so islands run in parallel
 * when the context has several evaluation threads, and the result does not depend on the thread count.
 * Islands keep more diversity than one panmictic population of the same total size; whether that
 * helps is problem-dependent and is measured, not assumed.
 */
public final class IslandGeneticAlgorithm implements SearchAlgorithm {

    private final Params params;
    private final GeneticAlgorithm ga;
    private final int islands;
    private final int islandSize;
    private final int migrationInterval;
    private final int migrants;
    private final boolean ring;
    private final boolean sendBest;

    public IslandGeneticAlgorithm(Params params) {
        this.islands = params.getInt("islands", 4);
        this.islandSize = params.getInt("islandSize", 25);
        this.migrationInterval = params.getInt("migrationInterval", 10);
        this.migrants = params.getInt("migrants", 2);
        String topology = params.getString("topology", "ring");
        String migrantSelection = params.getString("migrantSelection", "best");
        if (islands < 1) {
            throw new IllegalArgumentException("islands must be >= 1");
        }
        if (!List.of("ring", "complete").contains(topology)) {
            throw new IllegalArgumentException("topology must be 'ring' or 'complete'");
        }
        if (!List.of("best", "random").contains(migrantSelection)) {
            throw new IllegalArgumentException("migrantSelection must be 'best' or 'random'");
        }
        if (migrants < 0 || migrants >= islandSize) {
            throw new IllegalArgumentException("migrants must be in [0, islandSize)");
        }
        this.ring = topology.equals("ring");
        this.sendBest = migrantSelection.equals("best");
        this.params = params.with("populationSize", islandSize).with("replacement", "generational");
        this.ga = new GeneticAlgorithm(this.params);
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        SolutionSpace<S> space = context.space();
        MutationOperator<S> mutation = space.mutation(params);
        CrossoverOperator<S> crossover = space.crossover(params);

        List<S> initial = new ArrayList<>(islands * islandSize);
        for (int i = 0; i < islands * islandSize; i++) {
            initial.add(space.random(context.rng()));
        }
        List<List<Candidate<S>>> populations = split(context.evaluateAll(initial), islandSize);
        int generation = 0;
        int migrations = 0;
        record(context, populations, generation, migrations);
        int perIsland = ga.offspringPerGeneration();
        while (true) {
            List<S> children = new ArrayList<>(islands * perIsland);
            for (List<Candidate<S>> population : populations) {
                children.addAll(ga.breed(context, population, perIsland, mutation, crossover));
            }
            List<List<Candidate<S>>> offspring = split(context.evaluateAll(children), perIsland);
            for (int i = 0; i < islands; i++) {
                populations.set(i, ga.replace(populations.get(i), offspring.get(i)));
            }
            generation++;
            if (migrationInterval > 0 && islands > 1 && migrants > 0 && generation % migrationInterval == 0) {
                migrate(context, populations);
                migrations++;
            }
            context.nextIteration();
            record(context, populations, generation, migrations);
        }
    }

    /** One migration round; package-visible for tests. */
    <S> void migrate(SearchContext<S> context, List<List<Candidate<S>>> populations) {
        List<List<Candidate<S>>> emigrants = new ArrayList<>(islands);
        for (List<Candidate<S>> population : populations) {
            List<Candidate<S>> chosen = new ArrayList<>(migrants);
            if (sendBest) {
                List<Candidate<S>> sorted = new ArrayList<>(population);
                sorted.sort(Comparator.comparingDouble(Candidate::fitness));
                chosen.addAll(sorted.subList(0, migrants));
            } else {
                for (int m = 0; m < migrants; m++) {
                    chosen.add(population.get(context.rng().nextInt(population.size())));
                }
            }
            emigrants.add(chosen);
        }
        for (int target = 0; target < islands; target++) {
            List<Candidate<S>> incoming = new ArrayList<>();
            for (int source = 0; source < islands; source++) {
                boolean linked = ring ? (source + 1) % islands == target : source != target;
                if (linked) {
                    incoming.addAll(emigrants.get(source));
                }
            }
            List<Candidate<S>> population = new ArrayList<>(populations.get(target));
            population.sort(Comparator.comparingDouble(Candidate::fitness));
            int replace = Math.min(incoming.size(), population.size() - 1);
            for (int r = 0; r < replace; r++) {
                population.set(population.size() - 1 - r, incoming.get(r));
            }
            populations.set(target, population);
        }
    }

    private static <S> List<List<Candidate<S>>> split(List<Candidate<S>> all, int size) {
        List<List<Candidate<S>>> out = new ArrayList<>();
        for (int i = 0; i < all.size(); i += size) {
            out.add(new ArrayList<>(all.subList(i, Math.min(all.size(), i + size))));
        }
        return out;
    }

    private <S> void record(SearchContext<S> context, List<List<Candidate<S>>> populations, int generation,
                            int migrations) {
        List<Candidate<S>> all = new ArrayList<>();
        double bestOfBest = Double.POSITIVE_INFINITY;
        double worstOfBest = Double.NEGATIVE_INFINITY;
        double meanDiversity = 0;
        for (List<Candidate<S>> population : populations) {
            all.addAll(population);
            double best = Double.POSITIVE_INFINITY;
            for (Candidate<S> c : population) {
                best = Math.min(best, c.fitness());
            }
            bestOfBest = Math.min(bestOfBest, best);
            worstOfBest = Math.max(worstOfBest, best);
            meanDiversity += GeneticAlgorithm.diversity(context.space(), population) / populations.size();
        }
        ga.recordGeneration(context, all, generation);
        long x = context.evaluations();
        context.series("islandBestSpread", x, worstOfBest - bestOfBest);
        context.series("meanIslandDiversity", x, meanDiversity);
        context.stat("islands", populations.size());
        context.stat("migrations", migrations);
        context.stat("meanIslandDiversity", meanDiversity);
    }
}
