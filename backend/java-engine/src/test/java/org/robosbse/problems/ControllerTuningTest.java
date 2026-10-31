package org.robosbse.problems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
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
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;

class ControllerTuningTest {

    private static ControllerTuningProblem problem(String controller) {
        List<WorldMap> train = List.of(MapLibrary.builtin("obstacle-course"), MapLibrary.builtin("warehouse"));
        List<WorldMap> validation = List.of(MapLibrary.builtin("empty-field"));
        return new ControllerTuningProblem(train, validation, Params.of("controller", controller), Params.empty(),
                RobotSpec.DEFAULT);
    }

    @Test
    void defaultParametersDriveEveryTrainingMapSuccessfully() {
        for (String controller : List.of("pid-heading", "waypoint-follower", "pure-pursuit")) {
            ControllerTuningProblem p = problem(controller);
            Evaluation e = p.evaluate(p.defaults());
            assertEquals(1.0, e.metric("successRate", 0), controller + " defaults failed: " + e.metrics());
            assertTrue(e.feasible());
        }
    }

    @Test
    void tuningImprovesOnTheDefaultsAndReportsValidation() {
        ControllerTuningProblem p = problem("pid-heading");
        double baseline = p.evaluate(p.defaults()).fitness();
        SearchResult<RealVector> r = SearchAlgorithm.execute(
                AlgorithmRegistry.create("particle-swarm", Params.of("swarmSize", 15)),
                SearchContext.builder(p, Budget.evaluations(300), 3).build());
        assertTrue(r.best().fitness() < baseline, "tuned " + r.best().fitness() + " vs default " + baseline);
        Evaluation validation = p.evaluateOn(p.validation(), r.best().solution());
        assertTrue(validation.metric("successRate", 0) > 0, "tuned parameters must still work on the held-out map");
        for (int i = 0; i < p.names().size(); i++) {
            double[] range = ControllerTuningProblem.RANGES.get(p.names().get(i));
            double v = r.best().solution().get(i);
            assertTrue(v >= range[0] && v <= range[1], p.names().get(i) + " out of its tuning range");
        }
    }

    @Test
    void replayCoversTrainingAndValidationEpisodes() {
        ControllerTuningProblem p = problem("waypoint-follower");
        List<Map<String, Object>> episodes = p.replay(p.defaults());
        assertEquals(3, episodes.size());
        assertEquals("validation", episodes.get(2).get("split"));
    }

    @Test
    void unknownParametersAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new ControllerTuningProblem(
                List.of(MapLibrary.builtin("empty-field")), List.of(),
                Params.of("controller", "pid-heading", "parameters", "kp,turbo"), Params.empty(), RobotSpec.DEFAULT));
    }
}
