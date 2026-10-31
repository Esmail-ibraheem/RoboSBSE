package org.robosbse.problems.fleet;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.node.TextNode;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Trajectory;

class MultiRobotAllocationTest {

    private static MultiRobotAllocationProblem problem() {
        Fleet fleet = Fleet.resolve(TextNode.valueOf("deliveries-12"), null);
        return new MultiRobotAllocationProblem(fleet, MapLibrary.resolve(fleet.map(), null), Params.empty(), RobotSpec.DEFAULT);
    }

    @Test
    void randomKeysDecodeToRobotsAndServiceOrder() {
        MultiRobotAllocationProblem p = problem();
        int m = p.fleet().tasks().size();
        double[] g = new double[2 * m];
        for (int t = 0; t < m; t++) {
            g[t] = t % 3 + 0.4;
            g[m + t] = 1.0 - t / (double) m;
        }
        int[][] plan = p.decode(g);
        assertArrayEquals(new int[] {9, 6, 3, 0}, plan[0], "robot 0 serves its tasks by increasing priority");
        assertArrayEquals(new int[] {10, 7, 4, 1}, plan[1]);
        assertArrayEquals(new int[] {11, 8, 5, 2}, plan[2]);
        int[][] again = p.decode(p.encode(plan));
        for (int r = 0; r < 3; r++) {
            assertArrayEquals(plan[r], again[r]);
        }
    }

    @Test
    void costModelAddsUpTheRobots() {
        MultiRobotAllocationProblem p = problem();
        int[][] plan = p.greedy();
        Evaluation e = p.evaluate(plan);
        double makespan = 0;
        double distance = 0;
        int served = 0;
        for (int r = 0; r < plan.length; r++) {
            double[] c = p.robotCost(r, plan[r]);
            makespan = Math.max(makespan, c[1]);
            distance += c[0];
            served += plan[r].length;
            assertEquals(c[1], e.metric("time.robot" + r, -1), 1e-9);
        }
        assertEquals(p.fleet().tasks().size(), served, "greedy serves every task once");
        assertEquals(makespan, e.metric("makespan", -1), 1e-9);
        assertEquals(distance, e.metric("totalDistance", -1), 1e-9);
        assertEquals(makespan + 0.1 * distance, e.fitness(), 1e-9);
        assertEquals(e.fitness(), p.evaluate(new RealVector(p.encode(plan))).fitness(), 1e-9);
    }

    @Test
    void searchBeatsGreedyListScheduling() {
        MultiRobotAllocationProblem p = problem();
        double greedy = p.evaluate(p.greedy()).fitness();
        SearchResult<RealVector> r = SearchAlgorithm.execute(AlgorithmRegistry.create("genetic-algorithm",
                Params.of("populationSize", 50)), SearchContext.builder(p, Budget.evaluations(3000), 1).build());
        assertTrue(r.best().fitness() < greedy, r.best().fitness() + " vs greedy " + greedy);
    }

    @Test
    @SuppressWarnings("unchecked")
    void replaySimulatesEveryRobotCloseToTheModel() {
        MultiRobotAllocationProblem p = problem();
        int[][] plan = p.greedy();
        Map<String, Object> replay = p.replay(p.encode(plan));
        Map<String, Double> metrics = (Map<String, Double>) replay.get("metrics");
        assertEquals(0.0, metrics.get("failedLegs"));
        assertEquals("REACHED_GOAL", replay.get("outcome"));
        double model = metrics.get("makespan");
        double simulated = metrics.get("simulatedMakespan");
        assertTrue(Math.abs(simulated - model) / model < 0.1, "simulated " + simulated + " vs model " + model);
        List<Map<String, Object>> robots = (List<Map<String, Object>>) replay.get("robots");
        assertEquals(3, robots.size());
        assertEquals(metrics.get("conflicts"), (double) ((List<?>) replay.get("conflicts")).size());
    }

    @Test
    void conflictsAreFoundWhereTwoRobotsMeet() {
        MultiRobotAllocationProblem p = problem();
        Trajectory a = new Trajectory(0.1, false);
        a.hold(0, 0, 0, 0);
        a.hold(10, 10, 0, 0);
        Trajectory b = new Trajectory(0.1, false);
        b.hold(0, 10, 0, Math.PI);
        b.hold(10, 0, 0, Math.PI);
        Trajectory far = new Trajectory(0.1, false);
        far.hold(0, 5, 8, 0);
        List<Map<String, Object>> conflicts = p.conflicts(List.of(a, b, far));
        assertEquals(1, conflicts.size(), "only the head-on pair meets");
        Map<String, Object> c = conflicts.get(0);
        assertArrayEquals(new int[] {0, 1}, (int[]) c.get("robots"));
        assertEquals(5.0, (Double) c.get("x"), 0.6);
        assertTrue((Double) c.get("start") < 5 && (Double) c.get("end") > 5);
        assertTrue((Double) c.get("closestDistance") < 0.05);
    }
}
