package org.robosbse.problems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;

class NavigationProblemTest {

    private static NavigationProblem problem(String map, int waypoints) {
        return new NavigationProblem(MapLibrary.builtin(map), Params.of("waypoints", waypoints), Params.empty(),
                RobotSpec.DEFAULT);
    }

    @Test
    void straightRouteOnTheEmptyFieldSucceedsWithTheExpectedTime() {
        NavigationProblem p = problem("empty-field", 1);
        Evaluation e = p.evaluate(new RealVector(new double[] {10, 10}));
        assertEquals(1.0, e.metric("success", 0));
        assertEquals(0.0, e.metric("violation", -1));
        // Diagonal of 16*sqrt(2) = 22.6 m minus 0.3 m tolerance, plus turning and acceleration.
        assertTrue(e.fitness() > 22 && e.fitness() < 25.5, "time " + e.fitness());
    }

    @Test
    void routesThroughObstaclesArePenalisedAndFailuresRankBelowSuccesses() {
        NavigationProblem p = problem("obstacle-course", 1);
        Evaluation blocked = p.evaluate(new RealVector(new double[] {5, 10}));   // waypoint inside the pillar
        assertTrue(blocked.metric("violation", 0) > 0);
        assertTrue(blocked.fitness() > NavigationProblem.FAILURE_OFFSET);
        Map<String, Object> ref = p.reference();
        @SuppressWarnings("unchecked")
        Map<String, Object> astar = (Map<String, Object>) ref.get("astar");
        assertTrue((Boolean) astar.get("found"));
        assertTrue((Double) astar.get("fitness") < NavigationProblem.FAILURE_OFFSET, "A* route must execute successfully");
    }

    @Test
    void repairPushesWaypointsOutOfObstacles() {
        NavigationProblem p = problem("obstacle-course", 1);
        RealVector inside = new RealVector(new double[] {5.0, 10.4});
        RealVector repaired = p.repair(inside);
        assertTrue(new Vec2(repaired.get(0), repaired.get(1)).distanceTo(new Vec2(5, 10)) > 1.2 + 0.25,
                "waypoint must leave the inflated pillar");
        RealVector free = new RealVector(new double[] {2.0, 2.0});
        assertEquals(free, p.repair(free), "free waypoints are untouched");
    }

    @Test
    void evaluationIsDeterministic() {
        NavigationProblem p = problem("warehouse", 3);
        RealVector g = new RealVector(new double[] {11.5, 2, 11.5, 12, 20, 14.5});
        assertEquals(p.evaluate(g).fitness(), p.evaluate(g).fitness());
    }

    @Test
    void referenceRoutesOfAllBuiltInMapsExecuteWithTheWaypointFollower() {
        for (String name : List.of("empty-field", "obstacle-course", "warehouse", "maze")) {
            NavigationProblem p = problem(name, 3);
            @SuppressWarnings("unchecked")
            Map<String, Object> astar = (Map<String, Object>) p.reference().get("astar");
            @SuppressWarnings("unchecked")
            Map<String, Double> metrics = (Map<String, Double>) astar.get("metrics");
            assertEquals(1.0, metrics.get("success"), name + ": A* route failed in simulation");
        }
    }

    @Test
    void geneticAlgorithmFindsAFeasibleRouteOnTheWarehouse() {
        NavigationProblem p = problem("warehouse", 5);
        SearchResult<RealVector> r = SearchAlgorithm.execute(
                AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 40)),
                SearchContext.builder(p, Budget.evaluations(2000), 42).build());
        assertTrue(r.best().evaluation().metric("success", 0) > 0);
        assertTrue(r.firstTargetEvaluation() > 0);
        assertTrue(r.best().fitness() < 40, "travel time " + r.best().fitness());
    }
}
