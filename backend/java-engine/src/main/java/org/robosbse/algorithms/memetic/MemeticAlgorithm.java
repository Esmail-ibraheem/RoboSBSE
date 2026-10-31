package org.robosbse.algorithms.memetic;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.algorithms.evolutionary.GeneticAlgorithm;
import org.robosbse.core.Candidate;
import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SolutionSpace;

/**
 * Memetic algorithm: a genetic algorithm whose offspring are refined by a short local search.
 *
 * <p>Each generation the GA breeds and evaluates offspring as usual (selection, crossover, mutation;
 * generational or plus replacement). Each offspring, and each member of the initial population, is
 * then improved with probability {@code localRate} by one of three local searches, each limited to
 * {@code localBudget} evaluations:
 * <ul>
 *   <li><b>hill-climbing</b>: stochastic first improvement in the representation's neighbourhood
 *       (step {@code localStepSize} for real vectors);</li>
 *   <li><b>simulated-annealing</b>: Metropolis acceptance with a temperature that falls linearly from
 *       {@code T0 = localTemperature * max(1, |f(start)|)} to 0 over the local budget;</li>
 *   <li><b>problem</b>: stochastic first improvement in the problem's own neighbourhood
 *       ({@link org.robosbse.core.Problem#domainNeighbourhood()}, e.g. route smoothing for navigation).</li>
 * </ul>
 * The result of a local search is the best solution it evaluated. With <b>lamarckian</b> learning it
 * replaces the individual; with <b>baldwinian</b> learning the individual keeps its genotype but takes
 * the improved fitness, so selection favours individuals that are easy to improve without copying
 * the improvement into the population.
 *
 * <p>Every local-search evaluation goes through the shared context and is charged to the run's
 * budget, so a memetic algorithm and a GA with the same budget are directly comparable. Offspring are
 * evaluated as a batch (parallel when the context has several threads); local searches are sequential.
 */
public final class MemeticAlgorithm implements SearchAlgorithm {

    public static final List<String> LOCAL_SEARCHES = List.of("hill-climbing", "simulated-annealing", "problem");
    public static final List<String> LEARNING = List.of("lamarckian", "baldwinian");

    private final Params params;
    private final GeneticAlgorithm ga;
    private final String localSearch;
    private final double localRate;
    private final int localBudget;
    private final double localStepSize;
    private final double localTemperature;
    private final boolean lamarckian;

    public MemeticAlgorithm(Params params) {
        this.params = params;
        String replacement = params.getString("replacement", "generational");
        if (replacement.equals("steady-state")) {
            throw new IllegalArgumentException("the memetic algorithm supports generational or plus replacement");
        }
        this.ga = new GeneticAlgorithm(params);
        this.localSearch = params.getString("localSearch", "hill-climbing");
        this.localRate = params.getDouble("localRate", 0.1);
        this.localBudget = params.getInt("localBudget", 20);
        this.localStepSize = params.getDouble("localStepSize", 0.02);
        this.localTemperature = params.getDouble("localTemperature", 0.01);
        String learning = params.getString("learning", "lamarckian");
        if (!LOCAL_SEARCHES.contains(localSearch)) {
            throw new IllegalArgumentException("localSearch must be one of " + LOCAL_SEARCHES);
        }
        if (!LEARNING.contains(learning)) {
            throw new IllegalArgumentException("learning must be 'lamarckian' or 'baldwinian'");
        }
        if (localRate < 0 || localRate > 1) {
            throw new IllegalArgumentException("localRate must be in [0, 1]");
        }
        if (localBudget < 1) {
            throw new IllegalArgumentException("localBudget must be >= 1");
        }
        this.lamarckian = learning.equals("lamarckian");
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        SolutionSpace<S> space = context.space();
        MutationOperator<S> mutation = space.mutation(params);
        CrossoverOperator<S> crossover = space.crossover(params);
        Neighbourhood<S> neighbourhood = neighbourhood(context);
        Tally tally = new Tally();

        List<S> initial = new ArrayList<>(ga.populationSize());
        for (int i = 0; i < ga.populationSize(); i++) {
            initial.add(space.random(context.rng()));
        }
        List<Candidate<S>> population = refine(context, neighbourhood, context.evaluateAll(initial), tally);
        int generation = 0;
        record(context, population, generation, tally);
        while (true) {
            List<Candidate<S>> offspring = context.evaluateAll(
                    ga.breed(context, population, ga.offspringPerGeneration(), mutation, crossover));
            offspring = refine(context, neighbourhood, offspring, tally);
            population = ga.replace(population, offspring);
            generation++;
            context.nextIteration();
            record(context, population, generation, tally);
        }
    }

    private <S> Neighbourhood<S> neighbourhood(SearchContext<S> context) {
        if (localSearch.equals("problem")) {
            Neighbourhood<S> domain = context.problem().domainNeighbourhood();
            if (domain == null) {
                throw new IllegalArgumentException("localSearch = problem, but this problem defines no domain neighbourhood");
            }
            return domain;
        }
        return context.space().neighbourhood(params.with("stepSize", localStepSize));
    }

    /** Applies local search to each candidate with probability {@code localRate}. */
    private <S> List<Candidate<S>> refine(SearchContext<S> context, Neighbourhood<S> neighbourhood,
                                          List<Candidate<S>> candidates, Tally tally) {
        List<Candidate<S>> out = new ArrayList<>(candidates.size());
        for (Candidate<S> c : candidates) {
            if (localRate > 0 && context.rng().chance(localRate)) {
                out.add(improve(context, neighbourhood, c, tally));
            } else {
                out.add(c);
            }
        }
        return out;
    }

    /** One local search from {@code start}; returns the individual that enters the population. */
    <S> Candidate<S> improve(SearchContext<S> context, Neighbourhood<S> neighbourhood, Candidate<S> start, Tally tally) {
        long before = context.evaluations();
        Candidate<S> found = localSearch.equals("simulated-annealing")
                ? anneal(context, neighbourhood, start)
                : climb(context, neighbourhood, start);
        tally.searches++;
        tally.evaluations += context.evaluations() - before;
        if (found.fitness() < start.fitness()) {
            tally.improved++;
            tally.gain += start.fitness() - found.fitness();
        }
        if (lamarckian || found == start) {
            return found;
        }
        // Baldwinian: keep the genotype, inherit the fitness of what local search reached from it.
        return new Candidate<>(start.solution(), found.evaluation(), found.index());
    }

    private <S> Candidate<S> climb(SearchContext<S> context, Neighbourhood<S> neighbourhood, Candidate<S> start) {
        Candidate<S> current = start;
        for (int k = 0; k < localBudget; k++) {
            Candidate<S> next = context.evaluate(neighbourhood.random(current.solution(), context.rng()));
            if (next.fitness() < current.fitness()) {
                current = next;
            }
        }
        return current;
    }

    private <S> Candidate<S> anneal(SearchContext<S> context, Neighbourhood<S> neighbourhood, Candidate<S> start) {
        Rng rng = context.rng();
        double t0 = localTemperature * Math.max(1.0, Math.abs(start.fitness()));
        Candidate<S> current = start;
        Candidate<S> best = start;
        for (int k = 0; k < localBudget; k++) {
            Candidate<S> next = context.evaluate(neighbourhood.random(current.solution(), rng));
            double delta = next.fitness() - current.fitness();
            double temperature = t0 * (1.0 - (double) k / localBudget);
            if (delta <= 0 || (temperature > 0 && rng.nextDouble() < Math.exp(-delta / temperature))) {
                current = next;
            }
            if (current.fitness() < best.fitness()) {
                best = current;
            }
        }
        return best;
    }

    private <S> void record(SearchContext<S> context, List<Candidate<S>> population, int generation, Tally tally) {
        ga.recordGeneration(context, population, generation);
        context.stat("localSearches", tally.searches);
        context.stat("localEvaluations", tally.evaluations);
        context.stat("localSuccessRate", tally.searches == 0 ? 0 : (double) tally.improved / tally.searches);
        context.stat("localShare", context.evaluations() == 0 ? 0 : (double) tally.evaluations / context.evaluations());
        context.series("localSuccessRate", context.evaluations(),
                tally.searches == 0 ? 0 : (double) tally.improved / tally.searches);
        context.series("localMeanGain", context.evaluations(), tally.improved == 0 ? 0 : tally.gain / tally.improved);
    }

    /** Running totals of local-search activity. */
    static final class Tally {
        long searches;
        long evaluations;
        long improved;
        double gain;
    }
}
