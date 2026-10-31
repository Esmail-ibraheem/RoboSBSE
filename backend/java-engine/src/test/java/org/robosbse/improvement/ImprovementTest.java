package org.robosbse.improvement;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.RunControl;
import org.robosbse.experiments.SuiteRunner;
import org.robosbse.patching.MutationPoint;
import org.robosbse.patching.Patch;
import org.robosbse.patching.Subject;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;

class ImprovementTest {

    @Test
    void baselineScoresOneAndFewerHeadingCandidatesReduceWork() {
        try (ImprovementProblem p = new ImprovementProblem(Subject.resolve("gi-reactive-baseline", null),
                List.of(MapLibrary.builtin("empty-field"), MapLibrary.builtin("obstacle-course")),
                Params.of("tests", 6, "heldOutTests", 6, "poolSize", 30), RobotSpec.DEFAULT)) {
            Evaluation base = p.evaluate(Patch.empty());
            assertEquals(1.0, base.metric("valid", 0));
            assertEquals(p.baselineFitness(), base.fitness(), 1e-9);
            assertFalse(p.isTarget(base), "the baseline is not an improvement of itself");
            MutationPoint candidates = p.catalog().points().stream()
                    .filter(m -> m.method().equals("HEADING_CANDIDATES")).findFirst().orElseThrow();
            Patch fewer = Patch.of(List.of(new Patch.Edit(candidates.index(), candidates.alternatives().indexOf("18"))));
            Evaluation e = p.evaluate(fewer);
            assertTrue(e.metric("workRatio", 2) < 1.0, "half the candidate headings: less work per step");
            Patch none = Patch.of(List.of(new Patch.Edit(candidates.index(), candidates.alternatives().indexOf("0"))));
            Evaluation broken = p.evaluate(none);
            assertEquals(0.0, broken.metric("valid", 1), "no heading candidates: the controller cannot drive");
            assertTrue(broken.fitness() >= 10);
            assertTrue(p.catalog().points().stream().allMatch(m -> List.of("HEADING_CANDIDATES", "ARC_SAMPLES", "step",
                    "corridorFree", "safeSpeed", "arcClear").contains(m.method())), "edits stay inside the scope");
        }
    }

    @Test
    void suitesRunBenchmarksAndWriteAReport(@TempDir Path dir) {
        JsonNode bench = Json.parse("""
                {"name":"tiny-gp","runs":2,"budget":{"maxEvaluations":60},
                 "problem":{"type":"controller-synthesis","params":{"maps":"empty-field","testMaps":"obstacle-course"}},
                 "algorithms":[{"type":"genetic-algorithm","label":"GP","params":{"populationSize":10}},
                               {"type":"random-search","label":"Random programs"}],
                 "compareBy":"testFitness","reference":false}""");
        Json.write(dir.resolve("tiny.json"), bench);
        ObjectNode suite = (ObjectNode) Json.parse("{\"name\":\"tiny-suite\",\"items\":[{\"benchmark\":\"tiny.json\"}]}");
        ObjectNode out = SuiteRunner.run(suite, dir, new RunControl(), null, dir.resolve("out"));
        String report = SuiteRunner.report(out);
        assertTrue(report.contains("| GP | 2 |"), report);
        assertTrue(report.contains("mean test-map success"), report);
        assertTrue(Path.of(out.path("outputDir").asText()).resolve("report.md").toFile().exists());
    }
}
