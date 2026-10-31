package org.robosbse.experiments;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.robosbse.core.RunControl;
import org.robosbse.protocol.Json;

class ExperimentRunnerTest {

    static final Path REPO = Path.of(System.getProperty("robosbse.repoRoot", "../.."));

    private static JsonNode navConfig(String algorithm, long seed, int budget) {
        return Json.parse("""
                {"name":"t","seed":%d,"budget":{"maxEvaluations":%d},
                 "problem":{"type":"navigation","map":"builtin:obstacle-course","params":{"waypoints":3}},
                 "algorithm":{"type":"%s"}}""".formatted(seed, budget, algorithm));
    }

    @Test
    void resultIsSelfContainedAndWrittenToDisk(@TempDir Path dir) throws Exception {
        ObjectNode r = ExperimentRunner.run(navConfig("genetic-algorithm", 3, 400), null, new RunControl(), null,
                ExperimentRunner.Options.full(dir));
        assertEquals("robosbse.result/1", r.path("schema").asText());
        assertEquals(400, r.path("evaluations").asLong());
        assertTrue(r.path("experiment").path("problem").path("map").isObject(), "map must be inlined");
        assertTrue(r.path("replay").path("trajectory").path("x").size() > 10);
        assertTrue(r.path("reference").path("astar").path("found").asBoolean());
        assertTrue(r.path("convergence").size() > 0);
        assertTrue(r.path("series").has("diversity"));
        Path out = Path.of(r.path("outputDir").asText());
        assertTrue(Files.exists(out.resolve("result.json")));
        assertTrue(Files.exists(out.resolve("config.json")));
        assertTrue(Files.readAllLines(out.resolve("convergence.csv")).get(0).startsWith("evaluation,"));
    }

    @Test
    void rerunningTheEchoedConfigurationReproducesTheResult() {
        ObjectNode first = ExperimentRunner.run(navConfig("simulated-annealing", 5, 300), null, new RunControl(), null,
                ExperimentRunner.Options.summaryOnly());
        ObjectNode again = ExperimentRunner.run(first.path("experiment"), null, new RunControl(), null,
                ExperimentRunner.Options.summaryOnly());
        assertEquals(first.path("best").path("fitness").asDouble(), again.path("best").path("fitness").asDouble());
        assertEquals(first.path("convergence"), again.path("convergence"));
    }

    @Test
    void everyExampleConfigurationIsValid() throws Exception {
        try (var files = Files.list(REPO.resolve("examples/experiments"))) {
            for (Path f : files.filter(p -> p.toString().endsWith(".json")).toList()) {
                JsonNode cfg = Json.read(f);
                ObjectNode small = cfg.deepCopy();
                small.putObject("budget").put("maxEvaluations", 30);
                ObjectNode r = ExperimentRunner.run(small, f.getParent(), new RunControl(), null,
                        ExperimentRunner.Options.summaryOnly());
                assertEquals(30, r.path("evaluations").asLong(), f.getFileName().toString());
            }
        }
    }

    @Test
    void testingRunsProduceMinimisedFailureRecords() {
        JsonNode cfg = Json.parse("""
                {"name":"t","seed":7,"budget":{"maxEvaluations":800},
                 "problem":{"type":"controller-testing","map":"builtin:testing-arena",
                            "params":{"controller":"reactive-bug1-dropout","fitness":"collision"}},
                 "algorithm":{"type":"genetic-algorithm","params":{"populationSize":30}},
                 "options":{"minimizeFailures":true}}""");
        ObjectNode r = ExperimentRunner.run(cfg, null, new RunControl(), null, ExperimentRunner.Options.full(null));
        assertTrue(r.path("success").asBoolean());
        JsonNode failure = r.path("failures").get(0);
        assertEquals("robosbse.failure/1", failure.path("schema").asText());
        assertTrue(failure.path("minimization").path("simulations").asInt() > 0);
        assertTrue(r.path("replay").path("trajectory").path("rays").size() > 0, "testing replays include sensor rays");
    }

    @Test
    void benchmarkComparesAlgorithmsOnEqualBudgets(@TempDir Path dir) throws Exception {
        JsonNode bench = Json.parse("""
                {"name":"tiny","runs":3,"baseSeed":10,"threads":2,"budget":{"maxEvaluations":150},
                 "problem":{"type":"navigation","map":"builtin:warehouse","params":{"waypoints":3}},
                 "algorithms":[{"type":"random-search","label":"RS"},{"type":"genetic-algorithm","label":"GA",
                               "params":{"populationSize":20}}]}""");
        ObjectNode r = BenchmarkRunner.run(bench, null, new RunControl(), null, dir);
        assertEquals(6, r.path("runs").size());
        for (JsonNode run : r.path("runs")) {
            assertEquals(150, run.path("evaluations").asLong(), "equal budgets");
        }
        assertEquals(2, r.path("summary").size());
        assertEquals(1, r.path("pairwise").size());
        assertEquals(25, r.path("convergence").path("GA").size());
        assertFalse(r.path("reference").path("astar").has("trajectory"));
        assertFalse(Json.compact(r).contains("NaN"), "JSON output must not contain NaN");
        Path out = Path.of(r.path("outputDir").asText());
        for (String f : new String[] {"benchmark.json", "runs.csv", "summary.csv", "pairwise.csv", "convergence.csv"}) {
            assertTrue(Files.exists(out.resolve(f)), f);
        }
        // Same seeds, same budget -> identical per-run results when repeated (thread scheduling does not matter).
        ObjectNode again = BenchmarkRunner.run(bench, null, new RunControl(), null, null);
        for (int i = 0; i < 6; i++) {
            assertEquals(r.path("runs").get(i).path("bestFitness"), again.path("runs").get(i).path("bestFitness"));
        }
    }
    @Test
    void multiObjectiveRunsReportAParetoFrontWithHypervolume() {
        JsonNode cfg = Json.parse("""
                {"name":"mo","seed":2,"budget":{"maxEvaluations":800},
                 "problem":{"type":"navigation","map":"builtin:warehouse","params":{"waypoints":3}},
                 "algorithm":{"type":"nsga2","params":{"populationSize":20}}}""");
        ObjectNode r = ExperimentRunner.run(cfg, null, new RunControl(), null, ExperimentRunner.Options.summaryOnly());
        JsonNode pareto = r.path("pareto");
        assertEquals(List.of("time", "energy", "risk"), Json.convert(pareto.path("objectives"), List.class));
        assertEquals(3, pareto.path("referencePoint").size());
        double hv = pareto.path("hypervolume").asDouble();
        assertTrue(hv > 0 && hv <= 1, "normalised hypervolume " + hv);
        for (JsonNode p : pareto.path("points")) {
            assertEquals(3, p.path("objectives").size());
            assertTrue(p.path("feasible").asBoolean());
        }
        assertTrue(r.path("series").has("hypervolume"));
    }

    @Test
    void incompatibleAlgorithmsAreRejectedWithAClearMessage() {
        JsonNode pso = Json.parse("""
                {"name":"x","budget":{"maxEvaluations":10},
                 "problem":{"type":"warehouse-scheduling","params":{"tasks":"builtin:orders-4"}},
                 "algorithm":{"type":"particle-swarm"}}""");
        IllegalArgumentException e = org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class,
                () -> ExperimentRunner.run(pso, null, new RunControl(), null, ExperimentRunner.Options.summaryOnly()));
        assertTrue(e.getMessage().contains("permutation"), e.getMessage());
        JsonNode nsga = Json.parse("""
                {"name":"x","budget":{"maxEvaluations":10},
                 "problem":{"type":"controller-testing","map":"builtin:testing-arena"},
                 "algorithm":{"type":"nsga2"}}""");
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class,
                () -> ExperimentRunner.run(nsga, null, new RunControl(), null, ExperimentRunner.Options.summaryOnly()));
    }

    @Test
    void warehouseResultsInlineTheTaskSetAndRerunIdentically() {
        JsonNode cfg = Json.parse("""
                {"name":"wh","seed":4,"budget":{"maxEvaluations":400},
                 "problem":{"type":"warehouse-scheduling","params":{"tasks":"builtin:orders-8"}},
                 "algorithm":{"type":"genetic-algorithm","params":{"populationSize":20,"crossover":"order"}}}""");
        ObjectNode first = ExperimentRunner.run(cfg, null, new RunControl(), null, ExperimentRunner.Options.full(null));
        assertTrue(first.path("experiment").path("problem").path("params").path("tasks").isObject());
        assertTrue(first.path("replay").path("schedule").size() >= 18, "16 stops plus depot visits");
        assertTrue(first.path("reference").path("greedy").has("fitness"));
        ObjectNode again = ExperimentRunner.run(first.path("experiment"), null, new RunControl(), null,
                ExperimentRunner.Options.summaryOnly());
        assertEquals(first.path("best").path("fitness").asDouble(), again.path("best").path("fitness").asDouble());
    }

    @Test
    void tuningResultsCompareAgainstTheDefaultParameters() {
        JsonNode cfg = Json.parse("""
                {"name":"tune","seed":1,"budget":{"maxEvaluations":60},
                 "problem":{"type":"controller-tuning","params":{"controller":"waypoint-follower",
                            "maps":"obstacle-course,empty-field","validationMaps":"warehouse"}},
                 "algorithm":{"type":"random-search"}}""");
        ObjectNode r = ExperimentRunner.run(cfg, null, new RunControl(), null, ExperimentRunner.Options.full(null));
        assertTrue(r.path("reference").path("defaults").has("validation"));
        assertEquals(3, r.path("replay").path("episodes").size());
        assertTrue(r.path("replay").has("validation"));
        assertTrue(r.path("bestExtras").path("validationFitness").isNumber(), "generalisation of the best is reported");
    }
}
