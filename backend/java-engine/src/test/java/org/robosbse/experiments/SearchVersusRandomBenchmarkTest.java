package org.robosbse.experiments;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.robosbse.core.RunControl;
import org.robosbse.protocol.Json;

/**
 * Slow, statistical check of the main Phase 1 testing result (run with {@code gradlew benchmarkTest}):
 * on BUG-2, collision-guided genetic search discovers the failure in more runs than random testing
 * under the same budget, with a significant difference in final robustness.
 */
@Tag("benchmark")
class SearchVersusRandomBenchmarkTest {

    @Test
    void geneticSearchFindsBug2MoreOftenThanRandomTesting() {
        JsonNode config = Json.read(Path.of(System.getProperty("robosbse.repoRoot", "../.."))
                .resolve("examples/benchmarks/experiment-C-bug2.json"));
        ObjectNode bench = config.deepCopy();
        bench.put("threads", Math.max(1, Runtime.getRuntime().availableProcessors()));
        ObjectNode result = BenchmarkRunner.run(bench, null, new RunControl(), null, null);
        double randomRate = Double.NaN;
        double geneticRate = Double.NaN;
        for (JsonNode s : result.path("summary")) {
            if (s.path("algorithm").asText().equals("Random Testing")) {
                randomRate = s.path("successRate").asDouble();
            }
            if (s.path("algorithm").asText().equals("Genetic Search")) {
                geneticRate = s.path("successRate").asDouble();
            }
        }
        System.out.printf("success rate: random %.2f, genetic %.2f%n", randomRate, geneticRate);
        assertTrue(geneticRate > randomRate + 0.3, "genetic " + geneticRate + " vs random " + randomRate);
        for (JsonNode p : result.path("pairwise")) {
            if (p.path("a").asText().equals("Random Testing") && p.path("b").asText().equals("Genetic Search")) {
                assertTrue(p.path("pHolm").asDouble() < 0.05, "difference must be significant: " + p);
                assertTrue(p.path("a12").asDouble() < 0.29, "large effect in favour of genetic search: " + p);
            }
        }
    }
}
