package org.robosbse.algorithms.memetic;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Candidate;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;

class MemeticAlgorithmTest {

    private static SearchContext<RealVector> context(long budget) {
        return SearchContext.builder(new Sphere(3), Budget.evaluations(budget), 5).build();
    }

    @Test
    void localSearchEvaluationsAreChargedToTheBudget() {
        SearchContext<RealVector> ctx = context(1000);
        SearchResult<RealVector> r = SearchAlgorithm.execute(AlgorithmRegistry.create("memetic-algorithm",
                Params.of("localRate", 0.5, "localBudget", 10)), ctx);
        assertEquals(1000, r.evaluations());
        double localEvaluations = r.stats().get("localEvaluations");
        assertTrue(localEvaluations > 0 && localEvaluations < 1000);
        // Each offspring costs 1 evaluation plus, with probability 0.5, a 10-evaluation local search.
        assertEquals(5.0 / 6.0, r.stats().get("localShare"), 0.05);
    }

    @Test
    void lamarckianLearningWritesTheImprovementBackBaldwinianOnlyItsFitness() {
        RealVector start = new RealVector(new double[] {4, 4, 4});
        for (String learning : new String[] {"lamarckian", "baldwinian"}) {
            MemeticAlgorithm ma = new MemeticAlgorithm(Params.of("learning", learning, "localBudget", 40, "localStepSize", 0.05));
            SearchContext<RealVector> ctx = context(10_000);
            Candidate<RealVector> c = ctx.evaluate(start);
            Neighbourhood<RealVector> nb = ctx.space().neighbourhood(Params.of("stepSize", 0.05));
            Candidate<RealVector> refined = ma.improve(ctx, nb, c, new MemeticAlgorithm.Tally());
            assertTrue(refined.fitness() < c.fitness(), "40 hill-climbing steps must improve a point far from the optimum");
            assertEquals(41, ctx.evaluations());
            if (learning.equals("lamarckian")) {
                assertTrue(!refined.solution().equals(start));
                assertEquals(ctx.best().fitness(), refined.fitness());
            } else {
                assertSame(start, refined.solution());
                assertEquals(ctx.best().fitness(), refined.fitness());
            }
        }
    }

    @Test
    void simulatedAnnealingLocalSearchReturnsTheBestVisitedSolution() {
        MemeticAlgorithm ma = new MemeticAlgorithm(Params.of("localSearch", "simulated-annealing", "localBudget", 30,
                "localTemperature", 0.5));
        SearchContext<RealVector> ctx = context(10_000);
        Candidate<RealVector> c = ctx.evaluate(new RealVector(new double[] {3, 3, 3}));
        Candidate<RealVector> refined = ma.improve(ctx, ctx.space().neighbourhood(Params.of("stepSize", 0.05)), c,
                new MemeticAlgorithm.Tally());
        assertEquals(ctx.best().fitness(), Math.min(c.fitness(), refined.fitness()));
        assertTrue(refined.fitness() <= c.fitness());
    }

    @Test
    void problemLocalSearchNeedsADomainNeighbourhood() {
        SearchAlgorithm ma = AlgorithmRegistry.create("memetic-algorithm", Params.of("localSearch", "problem"));
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class,
                () -> SearchAlgorithm.execute(ma, context(100)));
        assertTrue(e.getMessage().contains("domain neighbourhood"));
    }

    @Test
    void steadyStateReplacementIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new MemeticAlgorithm(Params.of("replacement", "steady-state")));
    }
}
