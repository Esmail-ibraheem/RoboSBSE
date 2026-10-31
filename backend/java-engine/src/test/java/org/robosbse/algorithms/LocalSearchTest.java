package org.robosbse.algorithms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.local.CoolingSchedule;
import org.robosbse.algorithms.local.HillClimbing;
import org.robosbse.algorithms.local.SimulatedAnnealing;
import org.robosbse.core.Budget;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.StopReason;
import org.robosbse.core.representation.RealVector;

class LocalSearchTest {

    private static SearchResult<RealVector> run(SearchAlgorithm algorithm, int dim, long budget, long seed) {
        return SearchAlgorithm.execute(algorithm, SearchContext.builder(new Sphere(dim), Budget.evaluations(budget), seed).build());
    }

    @ParameterizedTest
    @ValueSource(strings = {"first", "best", "stochastic"})
    void hillClimbingConvergesOnTheSphere(String variant) {
        SearchResult<RealVector> r = run(new HillClimbing(Params.of("variant", variant, "stepSize", 0.002)), 3, 20_000, 1);
        assertTrue(r.best().fitness() < 1e-2, variant + " reached " + r.best().fitness());
    }

    @ParameterizedTest
    @ValueSource(strings = {"first", "best"})
    void hillClimbingOnlyMovesToStrictlyBetterSolutions(String variant) {
        SearchResult<RealVector> r = run(new HillClimbing(Params.of("variant", variant, "restarts", false, "stepSize", 0.01)),
                2, 50_000, 2);
        List<double[]> path = r.series().get("current");
        for (int i = 1; i < path.size(); i++) {
            assertTrue(path.get(i)[1] < path.get(i - 1)[1], "accepted a non-improving move");
        }
        assertEquals(StopReason.ALGORITHM_FINISHED, r.stopReason(), "without restarts the climber stops at the optimum");
        assertTrue(r.evaluations() < 50_000);
    }

    @Test
    void bestImprovementEvaluatesTheWholeNeighbourhood() {
        // From any interior point of a 2-D space the neighbourhood has 4 members; one climb step costs 4.
        SearchResult<RealVector> r = run(new HillClimbing(Params.of("variant", "best", "restarts", false, "stepSize", 0.01)),
                2, 50_000, 3);
        assertEquals(1 + 4 * (r.iterations() + 1), r.evaluations());
    }

    @Test
    void metropolisRuleAcceptsImprovementsAlways() {
        assertTrue(SimulatedAnnealing.accept(-1, 0, 0.999));
        assertTrue(SimulatedAnnealing.accept(0, 1, 0.999));
    }

    @Test
    void metropolisRuleUsesExpMinusDeltaOverT() {
        double p = Math.exp(-2.0 / 4.0);
        assertTrue(SimulatedAnnealing.accept(2, 4, p - 1e-9));
        assertFalse(SimulatedAnnealing.accept(2, 4, p + 1e-9));
        assertFalse(SimulatedAnnealing.accept(1e-9, 0, 0.0), "at T = 0 worse moves are never accepted");
    }

    @Test
    void metropolisAcceptanceFrequencyMatchesTheory() {
        Rng rng = new Rng(11);
        int accepted = 0;
        int n = 200_000;
        for (int i = 0; i < n; i++) {
            if (SimulatedAnnealing.accept(1.0, 1.0, rng.nextDouble())) {
                accepted++;
            }
        }
        assertEquals(Math.exp(-1), (double) accepted / n, 0.005);
    }

    @Test
    void coolingSchedulesDecrease() {
        for (String name : CoolingSchedule.NAMES) {
            CoolingSchedule c = CoolingSchedule.byName(name, 0.99, 0.01);
            double t = 10;
            for (int step = 1; step <= 100; step++) {
                double next = c.next(t, step, 10, 100);
                assertTrue(next <= t + 1e-12, name + " increased the temperature");
                t = next;
            }
        }
        assertEquals(0.0, CoolingSchedule.linear().next(5, 100, 10, 100), 1e-12, "linear reaches 0 at the end");
        assertEquals(10 * Math.pow(0.99, 1), CoolingSchedule.geometric(0.99).next(10, 1, 10, 100), 1e-12);
    }

    @Test
    void budgetDerivedCoolingReachesTheConfiguredFinalRatio() {
        SearchResult<RealVector> r = run(new SimulatedAnnealing(Params.of("initialTemperature", 5.0)), 2, 2_001, 4);
        assertEquals(5.0 * 1e-3, r.stats().get("temperature"), 5.0 * 1e-3 * 0.01);
    }

    @Test
    void simulatedAnnealingSolvesTheSphere() {
        SearchResult<RealVector> r = run(new SimulatedAnnealing(Params.of("stepSize", 0.02)), 3, 20_000, 5);
        assertTrue(r.best().fitness() < 0.05, "SA reached " + r.best().fitness());
        assertTrue(r.stats().get("worseAccepted") > 0, "SA must sometimes accept worse moves");
    }

    @Test
    void allAlgorithmsAreDeterministicForAFixedSeed() {
        for (AlgorithmRegistry.Descriptor d : AlgorithmRegistry.all()) {
            if (!d.supports("real-vector", false)) {
                continue;
            }
            SearchResult<RealVector> a = run(d.create(Params.empty()), 4, 600, 21);
            SearchResult<RealVector> b = run(d.create(Params.empty()), 4, 600, 21);
            assertEquals(a.best().solution(), b.best().solution(), d.id() + " is not deterministic");
            assertEquals(a.evaluations(), b.evaluations());
        }
    }

    @Test
    void everyAlgorithmBeatsItsFirstSampleAndRespectsTheBudget() {
        for (AlgorithmRegistry.Descriptor d : AlgorithmRegistry.all()) {
            if (!d.supports("real-vector", false)) {
                continue;
            }
            SearchResult<RealVector> r = run(d.create(Params.empty()), 4, 1_000, 8);
            assertTrue(r.evaluations() <= 1_000, d.id());
            assertTrue(r.convergence().size() > 1, d.id() + " never improved");
        }
    }
}
