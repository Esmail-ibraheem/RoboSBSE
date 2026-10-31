package org.robosbse.algorithms.evolutionary;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SolutionSpace;

/**
 * Genetic algorithm with configurable selection, crossover, mutation, elitism and replacement.
 *
 * <p>Each generation: select two parents, recombine them with probability {@code crossoverRate},
 * mutate every child, evaluate it. Replacement is either
 * <ul>
 *   <li><b>generational</b>: the {@code elitism} best parents survive unchanged and the rest of
 *       the population is replaced by offspring, or</li>
 *   <li><b>plus</b> ((mu+lambda) truncation): parents and offspring compete and the best
 *       {@code populationSize} survive (implicitly elitist), or</li>
 *   <li><b>steady-state</b>: offspring are produced two at a time and each replaces the current
 *       worst individual if it is better (so the population best never gets worse); a "generation"
 *       for statistics is {@code populationSize} offspring.</li>
 * </ul>
 * Per generation the best, mean and worst fitness and the genotypic diversity (mean pairwise
 * distance) are recorded. Offspring of a generation are evaluated as one batch
 * ({@link SearchContext#evaluateAll}), which runs in parallel when the context has several threads;
 * the random stream is consumed in the same order either way.
 */
public final class GeneticAlgorithm implements SearchAlgorithm {

    private final Params params;
    private final int populationSize;
    private final double crossoverRate;
    private final int elitism;
    private final String replacement;
    private final Selection selection;

    public GeneticAlgorithm(Params params) {
        this.params = params;
        this.populationSize = params.getInt("populationSize", 50);
        this.crossoverRate = params.getDouble("crossoverRate", 0.8);
        this.elitism = params.getInt("elitism", 2);
        this.replacement = params.getString("replacement", "generational");
        this.selection = Selection.byName(params.getString("selection", "tournament"),
                params.getInt("tournamentSize", 3), params.getDouble("rankPressure", 1.5));
        if (populationSize < 2) {
            throw new IllegalArgumentException("populationSize must be >= 2");
        }
        if (elitism < 0 || elitism >= populationSize) {
            throw new IllegalArgumentException("elitism must be in [0, populationSize)");
        }
        if (!List.of("generational", "plus", "steady-state").contains(replacement)) {
            throw new IllegalArgumentException("replacement must be 'generational', 'plus' or 'steady-state'");
        }
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        SolutionSpace<S> space = context.space();
        MutationOperator<S> mutation = space.mutation(params);
        CrossoverOperator<S> crossover = space.crossover(params);

        List<S> initial = new ArrayList<>(populationSize);
        for (int i = 0; i < populationSize; i++) {
            initial.add(space.random(context.rng()));
        }
        List<Candidate<S>> population = new ArrayList<>(context.evaluateAll(initial));
        int generation = 0;
        recordGeneration(context, population, generation);

        if (replacement.equals("steady-state")) {
            steadyState(context, population, mutation, crossover);
            return;
        }
        while (true) {
            int offspringNeeded = offspringPerGeneration();
            List<Candidate<S>> offspring = context.evaluateAll(breed(context, population, offspringNeeded, mutation, crossover));
            population = replace(population, offspring);
            generation++;
            context.nextIteration();
            recordGeneration(context, population, generation);
        }
    }

    private <S> void steadyState(SearchContext<S> context, List<Candidate<S>> population, MutationOperator<S> mutation,
                                 CrossoverOperator<S> crossover) {
        int produced = 0;
        int generation = 0;
        while (true) {
            S p1 = selection.select(population, context.rng()).solution();
            S p2 = selection.select(population, context.rng()).solution();
            List<S> children = context.rng().chance(crossoverRate) ? crossover.cross(p1, p2, context.rng()) : List.of(p1, p2);
            List<S> mutated = new ArrayList<>(children.size());
            for (S child : children) {
                mutated.add(mutation.mutate(child, context.rng()));
            }
            for (Candidate<S> c : context.evaluateAll(mutated)) {
                insertSteadyState(population, c);
                if (++produced % populationSize == 0) {
                    generation++;
                    context.nextIteration();
                    recordGeneration(context, population, generation);
                }
            }
        }
    }

    /**
     * Produces {@code count} offspring: selection of two parents, crossover with probability
     * {@code crossoverRate}, mutation of every child. Offspring are returned unevaluated.
     */
    public <S> List<S> breed(SearchContext<S> context, List<Candidate<S>> population, int count,
                             MutationOperator<S> mutation, CrossoverOperator<S> crossover) {
        List<S> offspring = new ArrayList<>(count);
        while (offspring.size() < count) {
            S p1 = selection.select(population, context.rng()).solution();
            S p2 = selection.select(population, context.rng()).solution();
            List<S> children = context.rng().chance(crossoverRate)
                    ? crossover.cross(p1, p2, context.rng())
                    : List.of(p1, p2);
            for (S child : children) {
                if (offspring.size() < count) {
                    offspring.add(mutation.mutate(child, context.rng()));
                }
            }
        }
        return offspring;
    }

    /** Steady-state replacement: the child replaces the worst member if it is better. */
    static <S> boolean insertSteadyState(List<Candidate<S>> population, Candidate<S> child) {
        int worst = 0;
        for (int i = 1; i < population.size(); i++) {
            if (population.get(i).fitness() > population.get(worst).fitness()) {
                worst = i;
            }
        }
        if (child.fitness() < population.get(worst).fitness()) {
            population.set(worst, child);
            return true;
        }
        return false;
    }

    /** Builds the next population (generational with elitism, or plus truncation). */
    public <S> List<Candidate<S>> replace(List<Candidate<S>> parents, List<Candidate<S>> offspring) {
        List<Candidate<S>> next = new ArrayList<>(populationSize);
        if (replacement.equals("generational")) {
            List<Candidate<S>> sortedParents = new ArrayList<>(parents);
            sortedParents.sort(Comparator.comparingDouble(Candidate::fitness));
            next.addAll(sortedParents.subList(0, elitism));
            next.addAll(offspring);
        } else {
            List<Candidate<S>> pool = new ArrayList<>(parents);
            pool.addAll(offspring);
            pool.sort(Comparator.comparingDouble(Candidate::fitness));
            next.addAll(pool.subList(0, populationSize));
        }
        return next;
    }

    public <S> void recordGeneration(SearchContext<S> context, List<Candidate<S>> population, int generation) {
        double best = Double.POSITIVE_INFINITY;
        double worst = Double.NEGATIVE_INFINITY;
        double sum = 0;
        for (Candidate<S> c : population) {
            best = Math.min(best, c.fitness());
            worst = Math.max(worst, c.fitness());
            sum += c.fitness();
        }
        double diversity = diversity(context.space(), population);
        long x = context.evaluations();
        context.series("populationBest", x, best);
        context.series("populationMean", x, sum / population.size());
        context.series("populationWorst", x, worst);
        context.series("diversity", x, diversity);
        context.stat("generation", generation);
        context.stat("diversity", diversity);
        context.stat("populationSize", population.size());
    }

    /** Mean pairwise genotypic distance of the population. */
    public static <S> double diversity(SolutionSpace<S> space, List<Candidate<S>> population) {
        int n = population.size();
        if (n < 2) {
            return 0;
        }
        double sum = 0;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                sum += space.distance(population.get(i).solution(), population.get(j).solution());
            }
        }
        return sum / (n * (n - 1) / 2.0);
    }

    public int populationSize() {
        return populationSize;
    }

    /** Offspring produced per generation: all but the elites (generational) or a full population (plus). */
    public int offspringPerGeneration() {
        return replacement.equals("generational") ? populationSize - elitism : populationSize;
    }
}
