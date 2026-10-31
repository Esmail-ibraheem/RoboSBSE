package org.robosbse.problems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.fasterxml.jackson.databind.node.TextNode;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.Permutation;
import org.robosbse.geometry.Vec2;
import org.robosbse.problems.warehouse.TaskLibrary;
import org.robosbse.problems.warehouse.TaskSet;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;

class WarehouseSchedulingTest {

    private static WarehouseSchedulingProblem problem(String tasks) {
        TaskSet set = TaskLibrary.resolve(TextNode.valueOf("builtin:" + tasks), null);
        return new WarehouseSchedulingProblem(set, MapLibrary.builtin("warehouse"), Params.empty(), RobotSpec.DEFAULT);
    }

    /** Checks every constraint the decoder promises on a decoded schedule. */
    private static void assertFeasible(WarehouseSchedulingProblem p, WarehouseSchedulingProblem.Schedule s) {
        TaskSet tasks = p.tasks();
        Set<Integer> picked = new HashSet<>();
        Set<Integer> delivered = new HashSet<>();
        int load = 0;
        double previousDeparture = 0;
        for (WarehouseSchedulingProblem.Visit v : s.visits()) {
            assertTrue(v.arrival() >= previousDeparture - 1e-9, "time must not run backwards");
            previousDeparture = v.departure();
            assertTrue(v.battery() >= tasks.reserve() - 1e-9, "battery reserve violated at " + v.label());
            switch (v.kind()) {
                case "pickup" -> {
                    for (String dep : tasks.tasks().get(v.task()).after()) {
                        int d = indexOf(tasks, dep);
                        assertTrue(delivered.contains(d), v.label() + " before its prerequisite " + dep);
                    }
                    picked.add(v.task());
                    load++;
                    assertTrue(load <= tasks.capacity(), "capacity exceeded at " + v.label());
                }
                case "delivery" -> {
                    assertTrue(picked.contains(v.task()), v.label() + " delivered before pickup");
                    delivered.add(v.task());
                    load--;
                }
                default -> {
                }
            }
            assertEquals(load, v.load());
        }
        assertEquals(tasks.tasks().size(), delivered.size(), "every task delivered");
        assertEquals("end", s.visits().get(s.visits().size() - 1).kind(), "the robot returns to the depot");
    }

    private static int indexOf(TaskSet tasks, String id) {
        for (int i = 0; i < tasks.tasks().size(); i++) {
            if (tasks.tasks().get(i).id().equals(id)) {
                return i;
            }
        }
        return -1;
    }

    @Test
    void everyPriorityListDecodesIntoAFeasibleSchedule() {
        WarehouseSchedulingProblem p = problem("orders-8");
        Rng rng = new Rng(1);
        boolean charged = false;
        for (int i = 0; i < 500; i++) {
            WarehouseSchedulingProblem.Schedule s = p.decode(p.space().random(rng));
            assertFeasible(p, s);
            charged |= s.charges() > 0;
        }
        assertTrue(charged, "the 35% battery of orders-8 forces charging");
        assertFeasible(p, p.decode(p.greedyOrder()));
    }

    @Test
    void geodesicDistancesAreSymmetricAndAtLeastEuclidean() {
        WarehouseSchedulingProblem p = problem("orders-8");
        List<Map<String, Object>> stops = p.stops();
        for (int i = 0; i < stops.size(); i++) {
            for (int j = 0; j < stops.size(); j++) {
                assertEquals(p.distance(i, j), p.distance(j, i), 0.25, "grid distances are symmetric up to discretisation");
                Vec2 a = new Vec2((Double) stops.get(i).get("x"), (Double) stops.get(i).get("y"));
                Vec2 b = new Vec2((Double) stops.get(j).get("x"), (Double) stops.get(j).get("y"));
                assertTrue(p.distance(i, j) >= a.distanceTo(b) - 0.2);
            }
        }
    }

    @Test
    void searchReachesTheExhaustiveOptimumOfTheSmallInstance() {
        WarehouseSchedulingProblem p = problem("orders-4");
        SearchResult<Permutation> exact = SearchAlgorithm.execute(AlgorithmRegistry.create("exhaustive-search", Params.empty()),
                SearchContext.builder(p, Budget.evaluations(100_000), 1).build());
        assertEquals(40_320, exact.evaluations());
        double optimum = exact.best().fitness();
        for (String alg : List.of("genetic-algorithm", "tabu-search", "simulated-annealing")) {
            Params params = alg.equals("tabu-search") ? Params.of("neighbourhood", "swap") : Params.empty();
            SearchResult<Permutation> r = SearchAlgorithm.execute(AlgorithmRegistry.create(alg, params),
                    SearchContext.builder(p, Budget.evaluations(3000), 2).build());
            assertTrue(r.best().fitness() <= optimum * 1.02, alg + " reached " + r.best().fitness() + " vs optimum " + optimum);
        }
        assertTrue(p.evaluate(p.greedyOrder()).fitness() >= optimum - 1e-9, "no heuristic beats the proven optimum");
    }

    @Test
    void replayExecutesTheMissionInTheSimulator() {
        WarehouseSchedulingProblem p = problem("orders-4");
        Map<String, Object> replay = p.replay(p.greedyOrder());
        assertEquals("REACHED_GOAL", replay.get("outcome"));
        @SuppressWarnings("unchecked")
        Map<String, Double> m = (Map<String, Double>) replay.get("metrics");
        assertEquals(0.0, m.get("failedLegs"));
        double modelled = m.get("modelledTravelTime");
        assertEquals(modelled, m.get("simulatedTravelTime"), 0.25 * modelled, "travel-time model within 25% of simulation");
    }

    @Test
    void invalidTaskSetsAreRejected() {
        TaskSet ok = TaskLibrary.resolve(TextNode.valueOf("builtin:orders-4"), null);
        List<TaskSet.Task> cyclic = List.of(
                new TaskSet.Task("A", ok.tasks().get(0).pickup(), ok.tasks().get(0).delivery(), 0, List.of("B")),
                new TaskSet.Task("B", ok.tasks().get(1).pickup(), ok.tasks().get(1).delivery(), 0, List.of("A")));
        TaskSet bad = new TaskSet(TaskSet.SCHEMA, "bad", null, ok.map(), ok.depot(), ok.charger(), 1, 0.8, 4, 60, 1, 0.1, cyclic);
        assertThrows(IllegalArgumentException.class,
                () -> new WarehouseSchedulingProblem(bad, MapLibrary.builtin("warehouse"), Params.empty(), RobotSpec.DEFAULT));
    }
}
