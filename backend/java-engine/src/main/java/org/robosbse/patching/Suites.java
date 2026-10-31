package org.robosbse.patching;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.Outcome;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;

/**
 * Test suites for patch evaluation. A test is a scenario of the testing arena plus an expectation:
 * no selected property may be violated, and when the trusted reference controller reaches the goal in
 * that scenario, the candidate must reach it too. The second condition stops trivial "patches" such
 * as never moving, which would otherwise pass every safety test.
 */
public final class Suites {

    /**
     * One test.
     *
     * @param requireGoal whether the candidate must reach the goal (the reference does)
     */
    public record TestCase(String name, Scenario scenario, boolean requireGoal) {
    }

    /** A performance episode on a map (genetic improvement). */
    public record Episode(String name, WorldMap map, int maxSteps) {
    }

    private Suites() {
    }

    public static Params testingParams(String properties, int maxSteps) {
        return Params.of("controller", "reactive", "properties", properties, "maxSteps", maxSteps, "simulationSeed", 1);
    }

    /** Valid random scenarios from the testing problem's scenario space (fixed seed, reproducible). */
    public static List<Scenario> randomScenarios(ControllerTestingProblem testing, long seed, int count) {
        Rng rng = new Rng(seed);
        List<Scenario> out = new ArrayList<>();
        for (int attempt = 0; out.size() < count && attempt < count * 20; attempt++) {
            Scenario s = testing.decode(testing.space().random(rng));
            if (testing.invalidity(s) == 0) {
                out.add(s);
            }
        }
        return out;
    }

    /**
     * Test cases judged by the reference controller ({@code reactive}): scenarios in which the reference
     * itself violates a property are dropped; {@code requireGoal} is whether the reference reached the goal.
     */
    public static List<TestCase> referenceCases(ControllerTestingProblem testing, List<Scenario> scenarios, String prefix) {
        List<TestCase> out = new ArrayList<>();
        for (int i = 0; i < scenarios.size(); i++) {
            ControllerTestingProblem.Execution ex = testing.execute(scenarios.get(i), "reactive", false);
            if (ex.violated().isEmpty()) {
                out.add(new TestCase(prefix + i, scenarios.get(i), ex.result().outcome() == Outcome.REACHED_GOAL));
            }
        }
        return out;
    }

    /** The {@code setup} message for a {@link org.robosbse.patching.sandbox.WorkerPool}. */
    public static ObjectNode setup(WorldMap arena, Params testingParams, RobotSpec robot, List<TestCase> tests,
                                   List<Episode> episodes) {
        ObjectNode setup = Json.object();
        setup.set("arena", Json.tree(arena));
        setup.set("testing", Json.tree(testingParams.asMap()));
        setup.set("robot", Json.tree(robot));
        ArrayNode t = setup.putArray("tests");
        for (TestCase c : tests) {
            ObjectNode n = t.addObject();
            n.put("name", c.name());
            n.set("scenario", Json.tree(c.scenario()));
            n.put("requireGoal", c.requireGoal());
        }
        ArrayNode e = setup.putArray("episodes");
        for (Episode ep : episodes) {
            ObjectNode n = e.addObject();
            n.put("name", ep.name());
            n.set("map", Json.tree(ep.map()));
            n.put("maxSteps", ep.maxSteps());
        }
        return setup;
    }

    /** Whether test {@code i} passed in a worker result (false for failed compilation, timeouts, crashes). */
    public static boolean passed(JsonNode result, int i) {
        return result.path("tests").path(i).path("passed").asBoolean(false);
    }

    /** Loss of test {@code i}: 0 when passed, 1 + severity when failed, 2 when the candidate did not run. */
    public static double loss(JsonNode result, int i) {
        JsonNode t = result.path("tests").path(i);
        return t.isMissingNode() ? 2.0 : t.path("loss").asDouble(2.0);
    }

    /** Whether a worker result represents a candidate that ran (compiled, no timeout, no crash). */
    public static boolean ran(JsonNode result) {
        return result.path("compiled").asBoolean(false) && !result.path("timeout").asBoolean(false)
                && !result.path("crashed").asBoolean(false) && result.path("tests").isArray();
    }

    /** Short status of a worker result. */
    public static String status(JsonNode result) {
        if (!result.path("compiled").asBoolean(false)) {
            return "does not compile";
        }
        if (result.path("timeout").asBoolean(false)) {
            return "timeout";
        }
        if (result.path("crashed").asBoolean(false)) {
            return "crashed";
        }
        return "ran";
    }
}
