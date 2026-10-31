package org.robosbse.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
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
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Vec2;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;

class ControllerTestingTest {

    private static ControllerTestingProblem problem(String controller, String properties, String fitness) {
        return new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                Params.of("controller", controller, "properties", properties, "fitness", fitness), Params.empty(),
                RobotSpec.DEFAULT);
    }

    /** An open, axis-aligned drive along y = 6 from x = 2 to x = 10 (no obstacles near the line). */
    private static Scenario openDrive(double battery, double speed) {
        return new Scenario(new Pose(2, 6, 0), new Vec2(10, 6), 0, 0, battery, speed, 1.0, List.of());
    }

    @Test
    void correctControllerPassesTheHandWrittenScenarios() {
        ControllerTestingProblem p = problem("reactive", "all", "min");
        ControllerTestingProblem.Execution ex = p.execute(openDrive(0.08, 1.0), "reactive", false);
        assertTrue(ex.violated().isEmpty(), "violations: " + ex.violated());
        assertTrue(ex.result().success());
    }

    @Test
    void bug3IsTriggeredByLowBatteryAndHighSpeedOnly() {
        ControllerTestingProblem p = problem("reactive-bug3-boost", "safety", "min");
        assertEquals(List.of("velocity"), p.execute(openDrive(0.08, 1.0), "reactive-bug3-boost", false).violated());
        assertTrue(p.execute(openDrive(0.5, 1.0), "reactive-bug3-boost", false).violated().isEmpty(), "battery ok");
        assertTrue(p.execute(openDrive(0.08, 0.6), "reactive-bug3-boost", false).violated().isEmpty(), "0.6*1.25 <= max");
    }

    @Test
    void bug4ReportsCompletionEarlyOnAnAxisAlignedApproach() {
        ControllerTestingProblem p = problem("reactive-bug4-completion", "safety", "min");
        ControllerTestingProblem.Execution ex = p.execute(openDrive(1.0, 0.8), "reactive-bug4-completion", false);
        assertEquals(List.of("completion"), ex.violated());
        assertTrue(ex.result().reportedDoneDistance() > 0.3);
        // A diagonal approach does not trigger it: the diamond lies inside the circle near the diagonals.
        Scenario diagonal = new Scenario(new Pose(2, 2, Math.PI / 4), new Vec2(9, 9), 0, 0, 1, 0.8, 1.0, List.of());
        assertTrue(p.execute(diagonal, "reactive-bug4-completion", false).violated().isEmpty());
    }

    @Test
    void decodingDropsObstaclesOnTheStartOrGoal() {
        ControllerTestingProblem p = problem("reactive", "safety", "min");
        double[] g = new double[p.space().dimension()];
        g[0] = 2;
        g[1] = 6;
        g[3] = 10;
        g[4] = 6;
        g[7] = 1;
        g[8] = 1;
        g[9] = 1;
        int b = ControllerTestingProblem.HEADER_GENES;
        // slot 0: circle on the start, slot 1: circle in the middle, both active
        double[][] slots = {{2, 6, 0.5, 0.1, 0.9}, {6, 6, 0.5, 0.1, 0.9}, {6, 2, 0.5, 0.1, 0.1}};
        for (int k = 0; k < slots.length; k++) {
            System.arraycopy(slots[k], 0, g, b + 5 * k, 5);
        }
        Scenario s = p.decode(new RealVector(g));
        assertEquals(List.of(new Circle(6, 6, 0.5)), s.obstacles(), "start-overlapping and inactive slots are dropped");
    }

    @Test
    void invalidScenariosAreWorseThanAnyValidOne() {
        ControllerTestingProblem p = problem("reactive", "safety", "min");
        Scenario onPillar = new Scenario(new Pose(4, 8, 0), new Vec2(10, 6), 0, 0, 1, 1, 1, List.of());
        Evaluation e = p.evaluateScenario(onPillar);
        assertFalse(e.feasible());
        assertTrue(e.fitness() > 1);
        assertFalse(p.isTarget(e));
    }

    @Test
    void robustnessIsNegativeExactlyWhenAPropertyIsViolated() {
        ControllerTestingProblem p = problem("reactive-bug1-dropout", "all", "min");
        Rng rng = new Rng(3);
        for (int i = 0; i < 150; i++) {
            Scenario s = p.decode(p.space().random(rng));
            if (p.invalidity(s) > 0) {
                continue;
            }
            for (Verdict v : p.execute(s, "reactive-bug1-dropout", false).verdicts()) {
                if (v.applicable()) {
                    assertEquals(v.violated(), v.robustness() < 0, v.property() + ": " + v.detail());
                }
            }
        }
    }

    @Test
    void correctControllerHasNoSafetyViolationsOnRandomScenarios() {
        ControllerTestingProblem p = problem("reactive", "safety", "min");
        Rng rng = new Rng(2024);
        int valid = 0;
        for (int i = 0; i < 400; i++) {
            Scenario s = p.decode(p.space().random(rng));
            if (p.invalidity(s) > 0) {
                continue;
            }
            valid++;
            assertEquals(List.of(), p.execute(s, "reactive", false).violated(), "scenario " + Json.compact(s));
        }
        assertTrue(valid > 250);
    }

    @Test
    void searchDiscoversBug1AndTheFailureReproducesFromTheSavedRecord(@TempDir Path dir) {
        ControllerTestingProblem p = problem("reactive-bug1-dropout", "safety", "collision");
        SearchResult<RealVector> r = SearchAlgorithm.execute(
                AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 30)),
                SearchContext.builder(p, Budget.evaluations(1500), 7).build());
        assertTrue(r.targetCount() > 0, "GA should discover BUG-1 within 1500 evaluations");
        Candidate<RealVector> first = r.targets().get(0);
        Scenario scenario = p.decode(first.solution());
        FailureRecord record = Failures.record(p, scenario, first.solution(), Map.of("seed", 7), null);
        assertTrue(record.violated().contains("collision"));
        assertEquals(List.of(), record.reference().get("violated"), "the correct controller passes the same scenario");

        Path file = dir.resolve("failure.json");
        Json.write(file, record);
        FailureRecord loaded = Json.convert(Json.read(file), FailureRecord.class);
        Failures.Reproduction again = Failures.reproduce(loaded, false);
        assertTrue(again.reproduced());
        assertEquals(record.violated(), again.violated());
    }

    @Test
    void minimisationKeepsTheFailureAndNeverAddsComplexity() {
        ControllerTestingProblem p = problem("reactive-bug1-dropout", "safety", "collision");
        Rng rng = new Rng(9);
        Scenario failing = null;
        for (int i = 0; i < 2000 && failing == null; i++) {
            Scenario s = p.decode(p.space().random(rng));
            if (p.invalidity(s) == 0 && p.execute(s, "reactive-bug1-dropout", false).violated().contains("collision")
                    && !s.obstacles().isEmpty()) {
                failing = s;
            }
        }
        assertTrue(failing != null, "random sampling should hit BUG-1 within 2000 scenarios");
        Failures.Minimized m = Failures.minimize(p, failing, "collision");
        assertTrue(p.execute(m.scenario(), "reactive-bug1-dropout", false).violated().contains("collision"));
        assertTrue(m.scenario().obstacles().size() <= failing.obstacles().size());
        assertTrue(m.scenario().sensorNoise() <= failing.sensorNoise());
        assertTrue(m.simulations() > 0);
    }

    static Stream<Path> regressionRecords() throws IOException {
        Path dir = Path.of(System.getProperty("robosbse.repoRoot", "../.."))
                .resolve("examples/faulty-controllers/regressions");
        List<Path> files = new ArrayList<>();
        try (Stream<Path> s = Files.list(dir)) {
            s.filter(f -> f.toString().endsWith(".json")).sorted().forEach(files::add);
        }
        return files.stream();
    }

    @ParameterizedTest
    @MethodSource("regressionRecords")
    void savedRegressionFailuresStillReproduce(Path file) {
        FailureRecord record = Json.convert(Json.read(file), FailureRecord.class);
        Failures.Reproduction r = Failures.reproduce(record, false);
        assertTrue(r.reproduced(), file.getFileName() + " no longer reproduces: " + r.violated());
        assertEquals(List.of(), r.referenceViolated(), file.getFileName() + ": correct controller must pass");
    }

    @Test
    void everyFaultyVariantHasARegressionRecord() throws IOException {
        List<String> controllers = new ArrayList<>();
        for (Path f : regressionRecords().toList()) {
            controllers.add(Json.read(f).path("controller").asText());
        }
        for (String id : List.of("reactive-bug1-dropout", "reactive-bug2-braking", "reactive-bug3-boost",
                "reactive-bug4-completion")) {
            assertTrue(controllers.contains(id), "missing regression record for " + id);
        }
    }
}
