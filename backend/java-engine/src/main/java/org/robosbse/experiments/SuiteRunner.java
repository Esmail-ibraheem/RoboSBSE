package org.robosbse.experiments;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.RunControl;
import org.robosbse.protocol.Json;

/**
 * Research experiment automation ({@code robosbse.suite/1}): runs a list of benchmarks and coverage
 * campaigns in order and writes one combined report, {@code suite.json} plus a Markdown summary
 * ({@code report.md}) with per-algorithm tables, significant pairwise differences and the problem-specific
 * outcomes recorded in each run (test-map generalisation, plausible / validated / equivalent patches,
 * validated improvements).
 *
 * <p>Configuration: {@code {"name", "description", "items": [{"benchmark": path} | {"coverage": path}]}},
 * paths relative to the suite file; an optional {@code threads} overrides every benchmark's run threads.
 */
public final class SuiteRunner {

    public static final String SCHEMA = "robosbse.suite-result/1";

    /** Progress callback per finished item. */
    public interface Listener {
        void onItem(int done, int total, String name);
    }

    private SuiteRunner() {
    }

    public static ObjectNode run(JsonNode suite, Path baseDir, RunControl control, Listener listener, Path outputDir) {
        Instant started = Instant.now();
        ObjectNode out = Json.object();
        out.put("schema", SCHEMA);
        out.put("name", suite.path("name").asText("suite"));
        out.put("description", suite.path("description").asText(""));
        out.put("startedAt", started.toString());
        ArrayNode items = out.putArray("items");
        Path dir = outputDir == null ? null : outputDir.resolve(
                ExperimentRunner.sanitize(suite.path("name").asText("suite")) + "-" + started.toString().replace(":", "").replaceAll("\\..*", ""));
        int total = suite.path("items").size();
        int done = 0;
        for (JsonNode item : suite.path("items")) {
            if (control.isCancelled()) {
                break;
            }
            ObjectNode entry = items.addObject();
            if (item.has("benchmark")) {
                Path file = resolve(baseDir, item.path("benchmark").asText());
                ObjectNode config = (ObjectNode) Json.read(file);
                if (suite.has("threads")) {
                    config.put("threads", suite.path("threads").asInt());
                }
                ObjectNode result = BenchmarkRunner.run(config, file.getParent(), control, null, dir);
                entry.put("kind", "benchmark");
                entry.put("file", file.toString());
                entry.set("result", light(result));
            } else if (item.has("coverage")) {
                Path file = resolve(baseDir, item.path("coverage").asText());
                ObjectNode result = CoverageCampaign.run(Json.read(file), file.getParent(), control, null, dir);
                entry.put("kind", "coverage");
                entry.put("file", file.toString());
                entry.set("result", result);
            } else {
                throw new IllegalArgumentException("suite items need 'benchmark' or 'coverage'");
            }
            done++;
            if (listener != null) {
                listener.onItem(done, total, entry.path("file").asText());
            }
        }
        out.put("finishedAt", Instant.now().toString());
        out.put("cancelled", control.isCancelled());
        if (dir != null) {
            Json.write(dir.resolve("suite.json"), out);
            write(dir.resolve("report.md"), report(out));
            out.put("outputDir", dir.toAbsolutePath().toString());
        }
        return out;
    }

    private static Path resolve(Path baseDir, String path) {
        Path p = Path.of(path);
        return p.isAbsolute() || baseDir == null ? p : baseDir.resolve(p).normalize();
    }

    /** Benchmark result without per-run convergence curves and solutions (they stay in the benchmark folder). */
    private static ObjectNode light(ObjectNode result) {
        ObjectNode copy = result.deepCopy();
        for (JsonNode run : copy.path("runs")) {
            ObjectNode r = (ObjectNode) run;
            r.remove("convergence");
            r.remove("checkpoints");
            r.remove("bestSolution");
            if (r.path("bestExtras").isObject()) {
                ObjectNode extras = (ObjectNode) r.path("bestExtras");
                extras.remove("diff");
                extras.remove("test");
            }
        }
        copy.remove("convergence");
        return copy;
    }

    /** Markdown report of a suite result. */
    public static String report(JsonNode suite) {
        StringBuilder md = new StringBuilder();
        md.append("# ").append(suite.path("name").asText()).append("\n\n");
        if (!suite.path("description").asText().isEmpty()) {
            md.append(suite.path("description").asText()).append("\n\n");
        }
        md.append("Started ").append(suite.path("startedAt").asText()).append(", finished ")
                .append(suite.path("finishedAt").asText()).append(".\n\n");
        for (JsonNode item : suite.path("items")) {
            JsonNode r = item.path("result");
            if (item.path("kind").asText().equals("coverage")) {
                md.append("## Coverage campaign: ").append(r.path("name").asText()).append("\n\n");
                JsonNode s = r.path("summary");
                md.append(String.format(Locale.ROOT, "Search-based: %d / %d outcomes; random testing: %s / %d.%n%n",
                        s.path("covered").asInt(), s.path("outcomes").asInt(), r.path("random").path("covered").asText("-"),
                        s.path("outcomes").asInt()));
                continue;
            }
            JsonNode bench = r.path("benchmark");
            md.append("## ").append(bench.path("name").asText()).append("\n\n");
            if (!bench.path("description").asText().isEmpty()) {
                md.append(bench.path("description").asText()).append("\n\n");
            }
            md.append(String.format(Locale.ROOT, "Budget %s evaluations, %d runs, compared by %s.%n%n",
                    bench.path("budget").path("maxEvaluations").asText(), r.path("runs").size(), r.path("compareBy").asText()));
            md.append("| Algorithm | Runs | Success | Median best | Median first target | Median ms | Outcome details |\n");
            md.append("|---|---|---|---|---|---|---|\n");
            for (JsonNode s : r.path("summary")) {
                String label = s.path("algorithm").asText();
                md.append(String.format(Locale.ROOT, "| %s | %d | %.0f%% | %s | %s | %s | %s |%n", label, s.path("runs").asInt(),
                        100 * s.path("successRate").asDouble(), num(s.path("median")),
                        num(s.path("medianFirstTargetEvaluation")), num(s.path("medianMillis")), details(label, r)));
            }
            md.append('\n');
            List<String> significant = new ArrayList<>();
            for (JsonNode p : r.path("pairwise")) {
                if (p.path("pHolm").asDouble(1) < 0.05) {
                    significant.add(String.format(Locale.ROOT, "%s vs %s: p(Holm) = %.3g, A12 = %.2f (%s)", p.path("a").asText(),
                            p.path("b").asText(), p.path("pHolm").asDouble(), p.path("a12").asDouble(), p.path("effect").asText()));
                }
            }
            md.append(significant.isEmpty() ? "No pairwise difference is significant after Holm correction.\n\n"
                    : "Significant after Holm correction: " + String.join("; ", significant) + ".\n\n");
        }
        return md.toString();
    }

    /** Problem-specific outcome counts from the runs' best-solution extras. */
    static String details(String label, JsonNode result) {
        int runs = 0;
        int plausible = 0;
        int heldOut = 0;
        int equivalent = 0;
        int identical = 0;
        int improvements = 0;
        int improvementsValidated = 0;
        double testSuccess = 0;
        int testRuns = 0;
        boolean repair = false;
        boolean gi = false;
        for (JsonNode run : result.path("runs")) {
            if (!run.path("algorithm").asText().equals(label)) {
                continue;
            }
            runs++;
            JsonNode e = run.path("bestExtras");
            if (e.has("plausible")) {
                repair = true;
                if (e.path("plausible").asBoolean()) {
                    plausible++;
                    heldOut += e.path("passesHeldOut").asBoolean() ? 1 : 0;
                    equivalent += e.path("equivalentToReferenceFix").asBoolean() ? 1 : 0;
                    identical += e.path("identicalToReferenceFix").asBoolean() ? 1 : 0;
                }
            }
            if (e.has("improvement")) {
                gi = true;
                if (e.path("improvement").asBoolean()) {
                    improvements++;
                    improvementsValidated += e.path("passesHeldOut").asBoolean() ? 1 : 0;
                }
            }
            if (e.has("testSuccessRate")) {
                testSuccess += e.path("testSuccessRate").asDouble();
                testRuns++;
            }
        }
        if (repair) {
            return String.format(Locale.ROOT, "plausible %d/%d, pass held-out %d, equivalent to fix %d, identical %d",
                    plausible, runs, heldOut, equivalent, identical);
        }
        if (gi) {
            return String.format(Locale.ROOT, "improved %d/%d, of which pass held-out %d", improvements, runs, improvementsValidated);
        }
        if (testRuns > 0) {
            return String.format(Locale.ROOT, "mean test-map success %.0f%%", 100 * testSuccess / testRuns);
        }
        return "";
    }

    private static String num(JsonNode v) {
        return v.isNumber() ? String.format(Locale.ROOT, "%.4g", v.asDouble()) : "-";
    }

    private static void write(Path path, String text) {
        try {
            Files.createDirectories(path.getParent());
            Files.writeString(path, text, StandardCharsets.UTF_8);
        } catch (java.io.IOException e) {
            throw new java.io.UncheckedIOException(e);
        }
    }
}
