package org.robosbse.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.fasterxml.jackson.databind.JsonNode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.robosbse.core.Params;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;

/**
 * Scenarios in which the correct reactive controller collided (obstacle surface between its 24 range rays).
 * The fixed ones must pass with the default perception (corner guard and 4-scan point memory) and still
 * collide with the configuration they were found with; the open one must still collide, so that the
 * documentation of the known issue stays true.
 */
class ReactivePerceptionTest {

    private static final Path ISSUES = Path.of("../../examples/faulty-controllers");

    static Stream<Path> fixedIssues() throws IOException {
        List<Path> files = new ArrayList<>();
        try (Stream<Path> s = Files.list(ISSUES.resolve("fixed-issues"))) {
            s.filter(f -> f.toString().endsWith(".json")).sorted().forEach(files::add);
        }
        assertEquals(7, files.size());
        return files.stream();
    }

    private static ControllerTestingProblem.Execution run(JsonNode record, Params controllerParams) {
        ControllerTestingProblem p = new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                Params.of("controller", "reactive", "properties", "safety", "fitness", "min", "maxSteps", 1200,
                        "simulationSeed", 1), controllerParams, RobotSpec.DEFAULT);
        return p.execute(Json.convert(record.get("scenario"), Scenario.class), "reactive", false);
    }

    @ParameterizedTest
    @MethodSource("fixedIssues")
    void fixedIssuesPassWithTheDefaultPerception(Path file) {
        assertEquals(List.of(), run(Json.read(file), Params.empty()).violated(), file.getFileName() + " violates safety");
    }

    @ParameterizedTest
    @MethodSource("fixedIssues")
    void fixedIssuesCollideWithTheConfigurationTheyWereFoundWith(Path file) {
        JsonNode record = Json.read(file);
        Params found = Params.of(Json.toMap(record.get("collidesWith")));
        assertTrue(run(record, found).violated().contains("collision"), file.getFileName() + ": expected the recorded collision");
    }

    @Test
    void theOpenKnownIssueStillCollides() {
        JsonNode record = Json.read(ISSUES.resolve("known-issues/reactive-residual-19.json"));
        assertTrue(run(record, Params.empty()).violated().contains("collision"),
                "the known issue no longer reproduces: update docs and move it to fixed-issues");
    }
}
