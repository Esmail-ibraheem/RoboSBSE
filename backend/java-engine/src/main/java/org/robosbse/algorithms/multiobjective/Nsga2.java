package org.robosbse.algorithms.multiobjective;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Hypervolume;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Params;
import org.robosbse.core.Pareto;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SolutionSpace;

/**
 * NSGA-II (Deb, Pratap, Agarwal and Meyarivan, 2002) with Deb's constrained domination.
 *
 * <p>Generation loop: binary tournaments on (non-domination rank, then crowding distance) select
 * parents; the representation's crossover and mutation create {@code populationSize} offspring;
 * parents and offspring are merged, sorted into non-dominated fronts and the next population is
 * filled front by front, the last partially fitting front being truncated by descending crowding
 * distance. Works for any representation; needs a problem with an objective vector.
 */
public final class Nsga2 implements SearchAlgorithm {

    /** An individual with its NSGA-II ranking data. */
    record Ranked<S>(Candidate<S> candidate, int rank, double crowding) {
    }

    private final Params params;
    private final int populationSize;
    private final double crossoverRate;

    public Nsga2(Params params) {
        this.params = params;
        this.populationSize = params.getInt("populationSize", 40);
        this.crossoverRate = params.getDouble("crossoverRate", 0.9);
        if (populationSize < 4) {
            throw new IllegalArgumentException("populationSize must be >= 4");
        }
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        if (context.problem().objectiveNames().isEmpty()) {
            throw new IllegalArgumentException("NSGA-II needs a multi-objective problem (no objectives declared by '"
                    + context.problem().type() + "')");
        }
        SolutionSpace<S> space = context.space();
        MutationOperator<S> mutation = space.mutation(params);
        CrossoverOperator<S> crossover = space.crossover(params);
        Rng rng = context.rng();
        List<S> randoms = new ArrayList<>();
        for (int i = 0; i < populationSize; i++) {
            randoms.add(space.random(rng));
        }
        List<Ranked<S>> population = environmentalSelection(context.evaluateAll(randoms), populationSize);
        double[] reference = context.problem().referencePoint();
        int generation = 0;
        while (true) {
            List<Candidate<S>> union = new ArrayList<>(2 * populationSize);
            for (Ranked<S> r : population) {
                union.add(r.candidate());
            }
            List<S> offspring = new ArrayList<>(populationSize);
            while (offspring.size() < populationSize) {
                S p1 = tournament(population, rng).candidate().solution();
                S p2 = tournament(population, rng).candidate().solution();
                List<S> children = rng.chance(crossoverRate) ? crossover.cross(p1, p2, rng) : List.of(p1, p2);
                for (S child : children) {
                    if (offspring.size() < populationSize) {
                        offspring.add(mutation.mutate(child, rng));
                    }
                }
            }
            union.addAll(context.evaluateAll(offspring));
            population = environmentalSelection(union, populationSize);
            generation++;
            context.nextIteration();
            record(context, population, generation, reference);
        }
    }

    /** Binary tournament: lower rank wins, then larger crowding distance, ties at random. */
    static <S> Ranked<S> tournament(List<Ranked<S>> population, Rng rng) {
        Ranked<S> a = population.get(rng.nextInt(population.size()));
        Ranked<S> b = population.get(rng.nextInt(population.size()));
        if (a.rank() != b.rank()) {
            return a.rank() < b.rank() ? a : b;
        }
        if (a.crowding() != b.crowding()) {
            return a.crowding() > b.crowding() ? a : b;
        }
        return rng.chance(0.5) ? a : b;
    }

    /** Selects {@code size} survivors by fronts and crowding distance; exposed for tests. */
    static <S> List<Ranked<S>> environmentalSelection(List<Candidate<S>> pool, int size) {
        List<Evaluation> evaluations = new ArrayList<>(pool.size());
        for (Candidate<S> c : pool) {
            evaluations.add(c.evaluation());
        }
        List<List<Integer>> fronts = Pareto.sort(evaluations);
        List<Ranked<S>> next = new ArrayList<>(size);
        for (int rank = 0; rank < fronts.size() && next.size() < size; rank++) {
            List<Integer> front = fronts.get(rank);
            double[] crowding = crowding(pool, front);
            Integer[] order = new Integer[front.size()];
            for (int i = 0; i < order.length; i++) {
                order[i] = i;
            }
            if (next.size() + front.size() > size) {
                Arrays.sort(order, Comparator.comparingDouble((Integer i) -> crowding[i]).reversed());
            }
            for (int i = 0; i < order.length && next.size() < size; i++) {
                next.add(new Ranked<>(pool.get(front.get(order[i])), rank, crowding[order[i]]));
            }
        }
        return next;
    }

    /**
     * Crowding distance within a front. Infeasible members are ordered by constraint violation
     * only, so they get a violation-based "crowding" that prefers less violated individuals.
     */
    private static <S> double[] crowding(List<Candidate<S>> pool, List<Integer> front) {
        boolean feasible = pool.get(front.get(0)).evaluation().feasible();
        if (!feasible) {
            double[] c = new double[front.size()];
            for (int i = 0; i < c.length; i++) {
                c[i] = -pool.get(front.get(i)).evaluation().constraintViolation();
            }
            return c;
        }
        List<double[]> points = new ArrayList<>(front.size());
        for (int idx : front) {
            points.add(pool.get(idx).evaluation().objectives());
        }
        return Pareto.crowdingDistance(points);
    }

    private static <S> void record(SearchContext<S> context, List<Ranked<S>> population, int generation, double[] ref) {
        int firstFront = 0;
        int feasible = 0;
        List<double[]> front = new ArrayList<>();
        for (Ranked<S> r : population) {
            if (r.candidate().evaluation().feasible()) {
                feasible++;
            }
            if (r.rank() == 0) {
                firstFront++;
                if (r.candidate().evaluation().feasible()) {
                    front.add(r.candidate().evaluation().objectives());
                }
            }
        }
        context.stat("generation", generation);
        context.stat("firstFrontSize", firstFront);
        context.stat("feasible", feasible);
        context.series("feasibleInPopulation", context.evaluations(), feasible);
        if (ref != null) {
            context.series("populationHypervolume", context.evaluations(), Hypervolume.normalised(front, ref));
        }
    }
}
