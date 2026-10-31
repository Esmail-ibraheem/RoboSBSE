package org.robosbse.experiments;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.RunControl;
import org.robosbse.core.representation.RealVector;
import org.robosbse.problems.NavigationProblem;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.controllers.ReactiveAvoidanceController;

class Phase3ExperimentsTest {

    static final Path REPO = ExperimentRunnerTest.REPO;

    @Test
    void navigationBehaviourDescriptorsAreNormalisedPathSamples() {
        NavigationProblem p = new NavigationProblem(MapLibrary.builtin("obstacle-course"), Params.of("waypoints", 3),
                Params.empty(), RobotSpec.DEFAULT);
        Evaluation e = p.evaluate(p.space().random(new org.robosbse.core.Rng(2)));
        double[] b = e.behaviour();
        assertEquals(8, b.length);
        for (double v : b) {
            assertTrue(v >= 0 && v <= 1, "component outside [0, 1]: " + v);
        }
    }

    @Test
    void routeSmoothingPullsAWaypointToTheMidpointOfItsNeighbours() {
        NavigationProblem p = new NavigationProblem(MapLibrary.builtin("obstacle-course"), Params.of("waypoints", 3),
                Params.empty(), RobotSpec.DEFAULT);
        RealVector g = new RealVector(new double[] {4, 4, 9, 2, 12, 12});
        RealVector smoothed = p.domainNeighbourhood().all(g).get(3); // waypoint 1, alpha = 1
        assertEquals((4 + 12) / 2.0, smoothed.get(2), 1e-12);
        assertEquals((4 + 12) / 2.0, smoothed.get(3), 1e-12);
        assertEquals(4, smoothed.get(0));
        assertEquals(6, p.domainNeighbourhood().all(g).size());
    }

    @Test
    void benchmarkAlgorithmsCanOverrideProblemParametersButNotCompareRawFitnessThen(@TempDir Path dir) {
        ObjectNode b = (ObjectNode) Json.parse("""
                {"name":"fitness-modes","runs":2,"budget":{"maxEvaluations":60},
                 "problem":{"type":"controller-testing","map":"builtin:testing-arena",
                            "params":{"controller":"reactive-bug3-boost","properties":"safety","maxSteps":300}},
                 "algorithms":[{"type":"random-search","label":"robustness"},
                               {"type":"random-search","label":"branch",
                                "problemParams":{"fitness":"branch:low-battery-boost:true"}}]}""");
        assertThrows(IllegalArgumentException.class, () -> BenchmarkRunner.run(b, null, new RunControl(), null, null));
        b.put("compareBy", "timeToTarget");
        ObjectNode out = BenchmarkRunner.run(b, null, new RunControl(), null, dir);
        assertEquals(4, out.path("runs").size());
        for (JsonNode run : out.path("runs")) {
            long t = run.path("timeToTarget").asLong();
            assertTrue(t >= 1 && t <= 61);
            boolean branch = run.path("algorithm").asText().equals("branch");
            assertEquals(branch, run.path("bestMetrics").has("approachLevel"));
        }
        assertEquals("timeToTarget", out.path("compareBy").asText());
    }

    @Test
    void coverageCampaignReportsEveryOutcomeAndARandomBaseline(@TempDir Path dir) throws Exception {
        JsonNode campaign = Json.parse("""
                {"name":"tiny","seed":2,"budget":{"maxEvaluations":150},
                 "problem":{"type":"controller-testing","map":"builtin:testing-arena",
                            "params":{"controller":"reactive","maxSteps":400}},
                 "algorithm":{"type":"genetic-algorithm","params":{"populationSize":10}}}""");
        ObjectNode out = CoverageCampaign.run(campaign, null, new RunControl(), null, dir);
        assertEquals(CoverageCampaign.SCHEMA, out.path("schema").asText());
        int outcomes = out.path("summary").path("outcomes").asInt();
        assertEquals(2 * ReactiveAvoidanceController.BRANCHES.size(), outcomes);
        assertEquals(outcomes, out.path("targets").size());
        assertTrue(out.path("summary").path("evaluations").asLong() <= 150);
        assertTrue(out.path("summary").path("covered").asInt() >= 20);
        for (JsonNode t : out.path("targets")) {
            String status = t.path("status").asText();
            assertTrue(status.equals("targeted") || status.equals("collateral") || status.equals("uncovered"), status);
            if (!status.equals("uncovered")) {
                assertEquals(30, t.path("scenario").size(), "covered targets carry a replayable genome");
            }
        }
        assertTrue(out.path("random").path("coverage").asDouble() > 0.5);
        assertTrue(Files.exists(Path.of(out.path("outputDir").asText()).resolve("coverage.csv")));
    }

    @Test
    void campaignsRefuseControllersWithoutInstrumentation() {
        JsonNode campaign = Json.parse("""
                {"budget":{"maxEvaluations":10},
                 "problem":{"type":"navigation","map":"builtin:obstacle-course"}}""");
        assertThrows(IllegalArgumentException.class, () -> CoverageCampaign.run(campaign, null, new RunControl(), null, null));
    }

    @Test
    void bundledPhase3ExamplesParse() throws Exception {
        for (String dirName : new String[] {"examples/coverage"}) {
            try (var files = Files.list(REPO.resolve(dirName))) {
                for (Path f : files.filter(x -> x.toString().endsWith(".json")).toList()) {
                    JsonNode node = Json.read(f);
                    assertTrue(node.path("problem").isObject(), f.toString());
                    Problems.create(node.path("problem"), f.getParent());
                }
            }
        }
    }
}
