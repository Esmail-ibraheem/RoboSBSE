package org.robosbse.algorithms.evolutionary;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Candidate;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;

class GeneticAlgorithmTest {

    private static SearchResult<RealVector> run(Params params, long budget, long seed) {
        GeneticAlgorithm ga = (GeneticAlgorithm) AlgorithmRegistry.create("genetic-algorithm", params);
        return SearchAlgorithm.execute(ga, SearchContext.builder(new Sphere(5), Budget.evaluations(budget), seed).build());
    }

    private static List<Candidate<String>> population(double... fitness) {
        List<Candidate<String>> pop = new ArrayList<>();
        for (int i = 0; i < fitness.length; i++) {
            pop.add(new Candidate<>("c" + i, Evaluation.of(fitness[i]), i + 1));
        }
        return pop;
    }

    @Test
    void solvesTheSphere() {
        SearchResult<RealVector> r = run(Params.of("populationSize", 40, "mutationSigma", 0.05), 20_000, 1);
        assertTrue(r.best().fitness() < 0.05, "GA reached " + r.best().fitness());
    }

    @Test
    void elitismNeverLosesThePopulationBest() {
        for (String replacement : List.of("generational", "plus")) {
            SearchResult<RealVector> r = run(Params.of("replacement", replacement, "elitism", 1), 5_000, 2);
            List<double[]> best = r.series().get("populationBest");
            for (int g = 1; g < best.size(); g++) {
                assertTrue(best.get(g)[1] <= best.get(g - 1)[1], replacement + ": best got worse at generation " + g);
            }
        }
    }

    @Test
    void withoutElitismTheBestCanBeLost() {
        // Not an invariant violation, just evidence that the elitism test above is meaningful.
        SearchResult<RealVector> r = run(Params.of("elitism", 0, "mutationSigma", 0.5, "mutationRate", 1.0), 5_000, 3);
        List<double[]> best = r.series().get("populationBest");
        boolean lost = false;
        for (int g = 1; g < best.size(); g++) {
            lost |= best.get(g)[1] > best.get(g - 1)[1];
        }
        assertTrue(lost);
    }

    @Test
    void populationSizeStaysConstant() {
        SearchResult<RealVector> r = run(Params.of("populationSize", 17, "elitism", 3), 2_000, 4);
        assertEquals(17.0, r.stats().get("populationSize"));
        GeneticAlgorithm ga = (GeneticAlgorithm) AlgorithmRegistry.create("genetic-algorithm",
                Params.of("populationSize", 4, "elitism", 1));
        List<Candidate<String>> parents = population(5, 1, 3, 2);
        List<Candidate<String>> offspring = population(9, 8, 7);
        List<Candidate<String>> next = ga.replace(parents, offspring);
        assertEquals(4, next.size());
        assertEquals(1.0, next.get(0).fitness(), "the elite is the best parent");
    }

    @Test
    void plusReplacementKeepsTheBestOfParentsAndOffspring() {
        GeneticAlgorithm ga = (GeneticAlgorithm) AlgorithmRegistry.create("genetic-algorithm",
                Params.of("populationSize", 3, "elitism", 0, "replacement", "plus"));
        List<Candidate<String>> next = ga.replace(population(5, 1, 9), population(2, 8, 0.5));
        List<Double> f = next.stream().map(Candidate::fitness).toList();
        assertEquals(List.of(0.5, 1.0, 2.0), f);
    }

    @Test
    void diversityIsRecordedAndShrinksAsThePopulationConverges() {
        SearchResult<RealVector> r = run(Params.of("populationSize", 30), 6_000, 5);
        List<double[]> diversity = r.series().get("diversity");
        assertTrue(diversity.get(diversity.size() - 1)[1] < diversity.get(0)[1]);
    }

    @Test
    void tournamentSelectionPressure() {
        Rng rng = new Rng(6);
        List<Candidate<String>> pop = population(4, 3, 2, 1, 0);
        Selection big = Selection.tournament(50);
        for (int i = 0; i < 100; i++) {
            assertEquals(0.0, big.select(pop, rng).fitness(), "a huge tournament always returns the best");
        }
        Map<Double, Integer> counts = new HashMap<>();
        Selection one = Selection.tournament(1);
        for (int i = 0; i < 50_000; i++) {
            counts.merge(one.select(pop, rng).fitness(), 1, Integer::sum);
        }
        for (int c : counts.values()) {
            assertEquals(10_000, c, 600, "tournament size 1 is uniform");
        }
    }

    @Test
    void rouletteFavoursBetterIndividualsUnderMinimisation() {
        Rng rng = new Rng(7);
        List<Candidate<String>> pop = population(10, -5, 3);
        Map<Double, Integer> counts = new HashMap<>();
        for (int i = 0; i < 30_000; i++) {
            counts.merge(Selection.roulette().select(pop, rng).fitness(), 1, Integer::sum);
        }
        assertTrue(counts.get(-5.0) > counts.get(3.0));
        assertTrue(counts.get(3.0) > counts.getOrDefault(10.0, 0));
    }

    @Test
    void rankSelectionGivesTheBestProbabilityPressureOverN() {
        Rng rng = new Rng(8);
        List<Candidate<String>> pop = population(1, 2, 3, 4);
        int best = 0;
        int n = 80_000;
        for (int i = 0; i < n; i++) {
            if (Selection.rank(1.8).select(pop, rng).fitness() == 1.0) {
                best++;
            }
        }
        assertEquals(1.8 / 4, (double) best / n, 0.01);
    }
}
