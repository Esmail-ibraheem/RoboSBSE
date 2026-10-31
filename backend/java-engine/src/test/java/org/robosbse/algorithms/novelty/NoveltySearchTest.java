package org.robosbse.algorithms.novelty;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;

class NoveltySearchTest {

    /**
     * Deceptive toy problem: fitness rewards x0 -> +5 but the global optimum (fitness -10) is a narrow
     * spot at x0 = -4.5, behind a slope that leads the other way. Behaviour = (x0, x1) scaled to [0, 1].
     */
    static final class Deceptive implements Problem<RealVector> {
        private final Sphere box = new Sphere(2);

        @Override
        public String type() {
            return "deceptive";
        }

        @Override
        public SolutionSpace<RealVector> space() {
            return box.space();
        }

        @Override
        public Evaluation evaluate(RealVector x) {
            double fitness = Math.hypot(x.get(0) + 4.5, x.get(1)) < 0.3 ? -10 : -x.get(0);
            return Evaluation.of(fitness).withBehaviour(new double[] {(x.get(0) + 5) / 10, (x.get(1) + 5) / 10});
        }

        @Override
        public boolean isTarget(Evaluation evaluation) {
            return evaluation.fitness() <= -10;
        }
    }

    @Test
    void noveltyIsTheMeanDistanceToTheKNearestNeighbours() {
        double[][] behaviours = {{0, 0}, {1, 0}, {0, 2}, {3, 4}};
        // From (0,0): distances 1, 2, 5 (pool) and 0.5 (archive).
        List<double[]> archive = List.of(new double[] {0.5, 0});
        assertEquals((0.5 + 1) / 2, NoveltySearch.novelty(behaviours, 0, archive, 2), 1e-12);
        assertEquals((0.5 + 1 + 2 + 5) / 4, NoveltySearch.novelty(behaviours, 0, archive, 10), 1e-12);
        assertEquals(0, NoveltySearch.novelty(new double[][] {{}, {1, 1}}, 0, List.of(), 3));
    }

    @Test
    void ranksScaleToTheUnitInterval() {
        assertArrayEquals(new double[] {0.5, 1, 0}, NoveltySearch.ranks(new double[] {2, 3, 1}, true), 1e-12);
        assertArrayEquals(new double[] {0.5, 0, 1}, NoveltySearch.ranks(new double[] {2, 3, 1}, false), 1e-12);
    }

    @Test
    void noveltySearchFillsItsArchiveAndCoversMoreBehaviourThanTheGaOnADeceptiveProblem() {
        double[] coverage = new double[2];
        int hitsNovelty = 0;
        int hitsGa = 0;
        for (long seed = 1; seed <= 5; seed++) {
            SearchResult<RealVector> ns = SearchAlgorithm.execute(AlgorithmRegistry.create("novelty-search", Params.empty()),
                    SearchContext.builder(new Deceptive(), Budget.evaluations(1500), seed).build());
            SearchResult<RealVector> ga = SearchAlgorithm.execute(AlgorithmRegistry.create("genetic-algorithm", Params.empty()),
                    SearchContext.builder(new Deceptive(), Budget.evaluations(1500), seed).build());
            assertTrue(ns.stats().get("archiveSize") > 0);
            assertTrue(ns.series().containsKey("meanNovelty"));
            coverage[0] += ns.stats().get("behaviourCoverage");
            coverage[1] += ga.stats().get("behaviourCoverage");
            hitsNovelty += ns.targetCount() > 0 ? 1 : 0;
            hitsGa += ga.targetCount() > 0 ? 1 : 0;
        }
        assertTrue(coverage[0] > coverage[1], "novelty " + coverage[0] + " vs GA " + coverage[1]);
        assertTrue(hitsNovelty >= hitsGa, "novelty found the hidden optimum " + hitsNovelty + "x, GA " + hitsGa + "x");
    }

    @Test
    void fitnessWeightOneIgnoresNovelty() {
        SearchResult<RealVector> r = SearchAlgorithm.execute(
                AlgorithmRegistry.create("novelty-search", Params.of("fitnessWeight", 1.0)),
                SearchContext.builder(new Sphere(3), Budget.evaluations(1500), 2).build());
        assertTrue(r.best().fitness() < 0.5, "with w = 1 the score is the fitness rank: " + r.best().fitness());
    }
}
