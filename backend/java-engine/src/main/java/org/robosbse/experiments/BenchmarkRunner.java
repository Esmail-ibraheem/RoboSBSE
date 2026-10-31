package org.robosbse.experiments;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.RunControl;
import org.robosbse.protocol.Json;

/**
 * Runs a benchmark ({@code robosbse.benchmark/1}): several algorithms, each over the same list of
 * random seeds and the same evaluation budget, on one problem. Produces per-run results, per-
 * algorithm summaries, median convergence curves and pairwise statistical comparisons, all computed
 * from the runs that were actually executed.
 */
public final class BenchmarkRunner {

    public static final String SCHEMA = "robosbse.benchmark-result/1";
    private static final int CHECKPOINTS = 25;
    private static final DateTimeFormatter STAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss").withZone(ZoneOffset.UTC);

    /** Progress callback (may be called from worker threads). */
    public interface Listener {
        void onRunFinished(int finished, int total, ObjectNode run);
    }

    private record RunSpec(int index, String label, JsonNode algorithm, long seed) {
    }

    /**
     * Run measures the pairwise tests can compare. {@code timeToTarget} is the evaluation of the first
     * target hit (e.g. the first failure), with runs that found none ranked after all that did
     * ({@code maxEvaluations + 1}, a censored value).
     */
    static final List<String> COMPARISONS = List.of("bestFitness", "hypervolume", "behaviourCoverage", "timeToTarget",
            "testFitness");

    /** The problem configuration with an algorithm's {@code problemParams} merged into its params. */
    static JsonNode withOverrides(JsonNode problem, JsonNode overrides) {
        if (!overrides.isObject() || overrides.isEmpty()) {
            return problem;
        }
        ObjectNode copy = problem.deepCopy();
        ObjectNode params = copy.path("params").isObject() ? (ObjectNode) copy.path("params") : copy.putObject("params");
        params.setAll((ObjectNode) overrides);
        return copy;
    }

    private BenchmarkRunner() {
    }

    public static ObjectNode run(JsonNode benchmark, Path baseDir, RunControl control, Listener listener, Path outputDir) {
        JsonNode problem = benchmark.path("problem");
        JsonNode budget = benchmark.path("budget");
        long maxEvaluations = budget.path("maxEvaluations").asLong(1000);
        List<Long> seeds = new ArrayList<>();
        if (benchmark.path("seeds").isArray()) {
            benchmark.path("seeds").forEach(s -> seeds.add(s.asLong()));
        } else {
            int runs = benchmark.path("runs").asInt(10);
            long base = benchmark.path("baseSeed").asLong(1);
            for (int i = 0; i < runs; i++) {
                seeds.add(base + i);
            }
        }
        String compareBy = benchmark.path("compareBy").asText("bestFitness");
        if (!COMPARISONS.contains(compareBy)) {
            throw new IllegalArgumentException("compareBy must be one of " + COMPARISONS);
        }
        List<RunSpec> specs = new ArrayList<>();
        List<String> labels = new ArrayList<>();
        for (JsonNode algorithm : benchmark.path("algorithms")) {
            if (algorithm.path("problemParams").isObject() && compareBy.equals("bestFitness")) {
                throw new IllegalArgumentException("per-algorithm problemParams change the fitness function, so best "
                        + "fitness values are not comparable; use compareBy timeToTarget, hypervolume or behaviourCoverage");
            }
            String label = algorithm.path("label").asText(algorithm.path("type").asText());
            if (labels.contains(label)) {
                throw new IllegalArgumentException("duplicate algorithm label '" + label + "'");
            }
            labels.add(label);
            for (long seed : seeds) {
                specs.add(new RunSpec(specs.size(), label, algorithm, seed));
            }
        }
        if (specs.isEmpty()) {
            throw new IllegalArgumentException("benchmark needs at least one algorithm and one seed");
        }

        int threads = Math.max(1, benchmark.path("threads").asInt(1));
        // Failure records (minimised when options.minimizeFailures) of the first failure of every run.
        boolean describeFailures = benchmark.path("describeFailures").asBoolean(false);
        ObjectNode[] results = new ObjectNode[specs.size()];
        AtomicInteger finished = new AtomicInteger();
        Instant started = Instant.now();
        ExecutorService pool = Executors.newFixedThreadPool(threads);
        try {
            List<Future<?>> futures = new ArrayList<>();
            for (RunSpec spec : specs) {
                futures.add(pool.submit(() -> {
                    if (control.isCancelled()) {
                        return;
                    }
                    ObjectNode config = Json.object();
                    config.put("schema", "robosbse.experiment/1");
                    config.put("name", benchmark.path("name").asText("benchmark"));
                    config.put("seed", spec.seed());
                    config.set("budget", budget);
                    config.set("problem", withOverrides(problem, spec.algorithm().path("problemParams")));
                    ObjectNode algorithm = config.putObject("algorithm");
                    algorithm.put("type", spec.algorithm().path("type").asText());
                    algorithm.set("params", spec.algorithm().path("params").isObject()
                            ? spec.algorithm().path("params") : Json.object());
                    ObjectNode options = benchmark.path("options").isObject()
                            ? ((ObjectNode) benchmark.path("options")).deepCopy() : Json.object();
                    if (spec.algorithm().path("options").isObject()) {
                        options.setAll((ObjectNode) spec.algorithm().path("options"));
                    }
                    if (describeFailures && !options.has("maxFailureRecords")) {
                        options.put("maxFailureRecords", 1);
                    }
                    config.set("options", options);
                    ObjectNode result = ExperimentRunner.run(config, baseDir, control, null,
                            describeFailures ? new ExperimentRunner.Options(false, false, true, null)
                                    : ExperimentRunner.Options.summaryOnly());
                    ObjectNode run = runSummary(spec, result, maxEvaluations);
                    results[spec.index()] = run;
                    if (listener != null) {
                        listener.onRunFinished(finished.incrementAndGet(), specs.size(), run);
                    }
                }));
            }
            for (Future<?> f : futures) {
                f.get();
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("benchmark interrupted", e);
        } catch (ExecutionException e) {
            Throwable cause = e.getCause();
            throw cause instanceof RuntimeException re ? re : new IllegalStateException(cause);
        } finally {
            pool.shutdownNow();
        }

        ObjectNode out = Json.object();
        out.put("schema", SCHEMA);
        out.put("engineVersion", ExperimentRunner.ENGINE_VERSION);
        out.set("benchmark", benchmark);
        out.put("startedAt", started.toString());
        out.put("finishedAt", Instant.now().toString());
        out.put("threads", threads);
        out.put("cancelled", control.isCancelled());
        ArrayNode runs = out.putArray("runs");
        for (ObjectNode r : results) {
            if (r != null) {
                runs.add(r);
            }
        }
        out.set("summary", summaries(labels, runs));
        out.set("convergence", medianConvergence(labels, runs));
        out.put("compareBy", compareBy);
        boolean largerIsBetter = compareBy.equals("hypervolume") || compareBy.equals("behaviourCoverage");
        out.set("pairwise", pairwise(labels, runs, compareBy, largerIsBetter));
        if (benchmark.path("reference").asBoolean(true)) {
            Map<String, Object> reference;
            try (ProblemInstance<?> instance = Problems.create(problem, baseDir)) {
                reference = instance.reference();
            }
            if (reference != null) {
                ObjectNode ref = (ObjectNode) Json.tree(reference);
                // Trajectories are only needed for single-run replays.
                ref.fields().forEachRemaining(e -> {
                    if (e.getValue().isObject()) {
                        ((ObjectNode) e.getValue()).remove("trajectory");
                    }
                });
                out.set("reference", ref);
            }
        }
        if (outputDir != null) {
            Path dir = write(outputDir, benchmark, out);
            out.put("outputDir", dir.toAbsolutePath().toString());
        }
        return out;
    }

    private static ObjectNode runSummary(RunSpec spec, ObjectNode result, long maxEvaluations) {
        ObjectNode run = Json.object();
        run.put("algorithm", spec.label());
        run.put("type", spec.algorithm().path("type").asText());
        run.put("seed", spec.seed());
        JsonNode best = result.path("best");
        Json.putFinite(run, "bestFitness", best.path("fitness").asDouble(Double.NaN));
        run.put("success", result.path("success").asBoolean());
        run.put("evaluations", result.path("evaluations").asLong());
        run.put("elapsedMs", result.path("elapsedMs").asLong());
        run.put("cpuMs", result.path("cpuMs").asLong());
        run.put("threads", result.path("threads").asInt(1));
        if (result.path("stats").has("behaviourCoverage")) {
            Json.putFinite(run, "behaviourCoverage", result.path("stats").path("behaviourCoverage").asDouble(Double.NaN));
        }
        run.put("stopReason", result.path("stopReason").asText());
        run.put("targetCount", result.path("targets").path("count").asLong());
        JsonNode extras = result.path("bestExtras");
        if (extras.isObject()) {
            run.set("bestExtras", extras);
        }
        if (extras.has("testFitness")) {
            Json.putFinite(run, "testFitness", extras.path("testFitness").asDouble(Double.NaN));
            Json.putFinite(run, "testSuccessRate", extras.path("testSuccessRate").asDouble(Double.NaN));
        }
        if (result.path("bestExtras").has("validationFitness")) {
            Json.putFinite(run, "validationFitness", result.path("bestExtras").path("validationFitness").asDouble(Double.NaN));
        }
        if (result.has("pareto")) {
            Json.putFinite(run, "hypervolume", result.path("pareto").path("hypervolume").asDouble(Double.NaN));
            run.put("paretoSize", result.path("pareto").path("points").size());
        }
        long firstTarget = result.path("targets").path("firstEvaluation").asLong(-1);
        run.put("firstTargetEvaluation", firstTarget);
        run.put("timeToTarget", firstTarget > 0 ? firstTarget : maxEvaluations + 1);
        JsonNode failures = result.path("failures");
        if (failures.isArray() && !failures.isEmpty()) {
            JsonNode f = failures.get(0);
            ObjectNode failure = run.putObject("firstFailure");
            failure.set("violated", f.path("violated"));
            failure.put("obstacles", f.path("scenario").path("obstacles").size());
            JsonNode m = f.path("minimization");
            if (m.isObject()) {
                failure.put("originalObstacles", m.path("originalObstacles").asInt());
                failure.put("minimisationSimulations", m.path("simulations").asInt());
                failure.set("minimisationSteps", m.path("steps"));
            }
        }
        run.set("bestMetrics", best.path("metrics"));
        run.set("bestSolution", best.path("solution"));
        ArrayNode checkpoints = run.putArray("checkpoints");
        JsonNode convergence = result.path("convergence");
        for (int k = 1; k <= CHECKPOINTS; k++) {
            long at = Math.max(1, Math.round((double) maxEvaluations * k / CHECKPOINTS));
            double value = Double.NaN;
            for (JsonNode p : convergence) {
                if (p.get(0).asDouble() > at) {
                    break;
                }
                value = p.get(1).asDouble();
            }
            checkpoints.add(Double.isFinite(value) ? Json.MAPPER.createArrayNode().add(at).add(value)
                    : Json.MAPPER.createArrayNode().add(at).addNull());
        }
        run.set("convergence", convergence);
        return run;
    }

    private static double[] column(ArrayNode runs, String label, String field) {
        List<Double> values = new ArrayList<>();
        for (JsonNode r : runs) {
            if (r.path("algorithm").asText().equals(label) && r.path(field).isNumber()) {
                values.add(r.path(field).asDouble());
            }
        }
        return values.stream().mapToDouble(Double::doubleValue).toArray();
    }

    private static ArrayNode summaries(List<String> labels, ArrayNode runs) {
        ArrayNode out = Json.MAPPER.createArrayNode();
        for (String label : labels) {
            double[] fitness = column(runs, label, "bestFitness");
            double[] time = column(runs, label, "elapsedMs");
            int successes = 0;
            List<Double> firstHits = new ArrayList<>();
            for (JsonNode r : runs) {
                if (!r.path("algorithm").asText().equals(label)) {
                    continue;
                }
                if (r.path("success").asBoolean()) {
                    successes++;
                }
                long first = r.path("firstTargetEvaluation").asLong(-1);
                if (first > 0) {
                    firstHits.add((double) first);
                }
            }
            Statistics.Summary s = Statistics.summarize(fitness);
            Statistics.Summary t = Statistics.summarize(time);
            Statistics.Summary cpu = Statistics.summarize(column(runs, label, "cpuMs"));
            Statistics.Summary h = Statistics.summarize(firstHits.stream().mapToDouble(Double::doubleValue).toArray());
            ObjectNode node = out.addObject();
            node.put("algorithm", label);
            node.put("runs", s.n());
            Json.putFinite(node, "mean", s.mean());
            Json.putFinite(node, "sd", s.sd());
            Json.putFinite(node, "min", s.min());
            Json.putFinite(node, "q1", s.q1());
            Json.putFinite(node, "median", s.median());
            Json.putFinite(node, "q3", s.q3());
            Json.putFinite(node, "max", s.max());
            node.put("successRate", s.n() == 0 ? 0 : (double) successes / s.n());
            Json.putFinite(node, "meanMillis", t.mean());
            Json.putFinite(node, "medianMillis", t.median());
            Json.putFinite(node, "medianCpuMillis", cpu.median());
            Json.putFinite(node, "medianFirstTargetEvaluation", h.median());
            node.put("runsWithTarget", h.n());
            double[] validation = column(runs, label, "validationFitness");
            if (validation.length > 0) {
                Statistics.Summary v = Statistics.summarize(validation);
                ObjectNode valNode = node.putObject("validation");
                Json.putFinite(valNode, "median", v.median());
                Json.putFinite(valNode, "mean", v.mean());
                Json.putFinite(valNode, "min", v.min());
                Json.putFinite(valNode, "max", v.max());
            }
            List<Double> before = new ArrayList<>();
            List<Double> after = new ArrayList<>();
            for (JsonNode r : runs) {
                JsonNode f = r.path("firstFailure");
                if (r.path("algorithm").asText().equals(label) && f.has("originalObstacles")) {
                    before.add(f.path("originalObstacles").asDouble());
                    after.add(f.path("obstacles").asDouble());
                }
            }
            if (!after.isEmpty()) {
                ObjectNode min = node.putObject("minimisedFailures");
                min.put("count", after.size());
                Json.putFinite(min, "medianObstaclesBefore",
                        Statistics.summarize(before.stream().mapToDouble(Double::doubleValue).toArray()).median());
                Json.putFinite(min, "medianObstaclesAfter",
                        Statistics.summarize(after.stream().mapToDouble(Double::doubleValue).toArray()).median());
            }
            double[] testFitness = column(runs, label, "testFitness");
            if (testFitness.length > 0) {
                Statistics.Summary v = Statistics.summarize(testFitness);
                ObjectNode testNode = node.putObject("test");
                Json.putFinite(testNode, "median", v.median());
                Json.putFinite(testNode, "mean", v.mean());
                Json.putFinite(testNode, "min", v.min());
                Json.putFinite(testNode, "max", v.max());
                Statistics.Summary rate = Statistics.summarize(column(runs, label, "testSuccessRate"));
                Json.putFinite(testNode, "meanSuccessRate", rate.mean());
            }
            double[] coverage = column(runs, label, "behaviourCoverage");
            if (coverage.length > 0) {
                Statistics.Summary v = Statistics.summarize(coverage);
                ObjectNode covNode = node.putObject("behaviourCoverage");
                Json.putFinite(covNode, "mean", v.mean());
                Json.putFinite(covNode, "median", v.median());
                Json.putFinite(covNode, "min", v.min());
                Json.putFinite(covNode, "max", v.max());
            }
            double[] hv = column(runs, label, "hypervolume");
            if (hv.length > 0) {
                Statistics.Summary v = Statistics.summarize(hv);
                ObjectNode hvNode = node.putObject("hypervolume");
                Json.putFinite(hvNode, "mean", v.mean());
                Json.putFinite(hvNode, "sd", v.sd());
                Json.putFinite(hvNode, "min", v.min());
                Json.putFinite(hvNode, "q1", v.q1());
                Json.putFinite(hvNode, "median", v.median());
                Json.putFinite(hvNode, "q3", v.q3());
                Json.putFinite(hvNode, "max", v.max());
            }
        }
        return out;
    }

    private static ObjectNode medianConvergence(List<String> labels, ArrayNode runs) {
        ObjectNode out = Json.object();
        for (String label : labels) {
            List<double[]> perRun = new ArrayList<>();
            for (JsonNode r : runs) {
                if (!r.path("algorithm").asText().equals(label)) {
                    continue;
                }
                double[] values = new double[CHECKPOINTS];
                for (int k = 0; k < CHECKPOINTS; k++) {
                    values[k] = r.path("checkpoints").path(k).path(1).asDouble(Double.NaN);
                }
                perRun.add(values);
            }
            ArrayNode curve = out.putArray(label);
            if (perRun.isEmpty()) {
                continue;
            }
            for (int k = 0; k < CHECKPOINTS; k++) {
                List<Double> col = new ArrayList<>();
                for (double[] v : perRun) {
                    if (!Double.isNaN(v[k])) {
                        col.add(v[k]);
                    }
                }
                double at = runs.get(0).path("checkpoints").path(k).path(0).asDouble();
                Statistics.Summary s = Statistics.summarize(col.stream().mapToDouble(Double::doubleValue).toArray());
                ArrayNode row = curve.addArray().add(at);
                for (double v : new double[] {s.median(), s.q1(), s.q3()}) {
                    if (Double.isFinite(v)) {
                        row.add(v);
                    } else {
                        row.addNull();
                    }
                }
            }
        }
        return out;
    }

    /**
     * Pairwise Mann-Whitney U and A12 on a per-run metric. For metrics where larger is better
     * (hypervolume) the values are negated first, so A12 always means "A better than B".
     */
    private static ArrayNode pairwise(List<String> labels, ArrayNode runs, String field, boolean largerIsBetter) {
        List<String[]> pairs = new ArrayList<>();
        List<Double> pValues = new ArrayList<>();
        List<double[]> stats = new ArrayList<>();
        for (int i = 0; i < labels.size(); i++) {
            for (int j = i + 1; j < labels.size(); j++) {
                double[] a = column(runs, labels.get(i), field);
                double[] b = column(runs, labels.get(j), field);
                if (largerIsBetter) {
                    for (int k = 0; k < a.length; k++) {
                        a[k] = -a[k];
                    }
                    for (int k = 0; k < b.length; k++) {
                        b[k] = -b[k];
                    }
                }
                Statistics.MannWhitney mw = Statistics.mannWhitney(a, b);
                pairs.add(new String[] {labels.get(i), labels.get(j)});
                pValues.add(Double.isNaN(mw.pValue()) ? 1.0 : mw.pValue());
                stats.add(new double[] {mw.u(), mw.z(), mw.pValue(), Statistics.a12(a, b)});
            }
        }
        double[] holm = Statistics.holm(pValues.stream().mapToDouble(Double::doubleValue).toArray());
        ArrayNode out = Json.MAPPER.createArrayNode();
        for (int k = 0; k < pairs.size(); k++) {
            ObjectNode node = out.addObject();
            node.put("a", pairs.get(k)[0]);
            node.put("b", pairs.get(k)[1]);
            Json.putFinite(node, "u", stats.get(k)[0]);
            Json.putFinite(node, "z", stats.get(k)[1]);
            Json.putFinite(node, "pValue", stats.get(k)[2]);
            Json.putFinite(node, "pHolm", holm[k]);
            Json.putFinite(node, "a12", stats.get(k)[3]);
            node.put("effect", Statistics.a12Magnitude(stats.get(k)[3]));
        }
        return out;
    }

    private static Path write(Path root, JsonNode benchmark, ObjectNode out) {
        String name = ExperimentRunner.sanitize(benchmark.path("name").asText("benchmark"));
        Path dir = root.resolve(STAMP.format(Instant.now()) + "-benchmark-" + name);
        int suffix = 1;
        while (Files.exists(dir)) {
            dir = root.resolve(STAMP.format(Instant.now()) + "-benchmark-" + name + "-" + suffix++);
        }
        Json.write(dir.resolve("benchmark.json"), out);
        List<String> runLines = new ArrayList<>();
        runLines.add("algorithm,type,seed,best_fitness,success,evaluations,elapsed_ms,first_target_evaluation,target_count,stop_reason,hypervolume,cpu_ms,threads,behaviour_coverage");
        for (JsonNode r : out.path("runs")) {
            runLines.add(String.join(",", ExperimentRunner.Csv.escape(r.path("algorithm").asText()),
                    r.path("type").asText(), r.path("seed").asText(), num(r.path("bestFitness")),
                    r.path("success").asText(), r.path("evaluations").asText(), r.path("elapsedMs").asText(),
                    r.path("firstTargetEvaluation").asText(), r.path("targetCount").asText(),
                    r.path("stopReason").asText(), num(r.path("hypervolume")), r.path("cpuMs").asText(),
                    r.path("threads").asText(), num(r.path("behaviourCoverage"))));
        }
        ExperimentRunner.Csv.write(dir.resolve("runs.csv"), runLines);
        List<String> summaryLines = new ArrayList<>();
        summaryLines.add("algorithm,runs,mean,sd,min,q1,median,q3,max,success_rate,mean_ms,median_first_target_evaluation,median_ms,median_cpu_ms");
        for (JsonNode s : out.path("summary")) {
            summaryLines.add(String.join(",", ExperimentRunner.Csv.escape(s.path("algorithm").asText()),
                    s.path("runs").asText(), num(s.path("mean")), num(s.path("sd")), num(s.path("min")),
                    num(s.path("q1")), num(s.path("median")), num(s.path("q3")), num(s.path("max")),
                    num(s.path("successRate")), num(s.path("meanMillis")), num(s.path("medianFirstTargetEvaluation")),
                    num(s.path("medianMillis")), num(s.path("medianCpuMillis"))));
        }
        ExperimentRunner.Csv.write(dir.resolve("summary.csv"), summaryLines);
        List<String> pairLines = new ArrayList<>();
        pairLines.add("a,b,u,z,p_value,p_holm,a12,effect");
        for (JsonNode p : out.path("pairwise")) {
            pairLines.add(String.join(",", ExperimentRunner.Csv.escape(p.path("a").asText()),
                    ExperimentRunner.Csv.escape(p.path("b").asText()), num(p.path("u")), num(p.path("z")),
                    num(p.path("pValue")), num(p.path("pHolm")), num(p.path("a12")), p.path("effect").asText()));
        }
        ExperimentRunner.Csv.write(dir.resolve("pairwise.csv"), pairLines);
        List<String> convLines = new ArrayList<>();
        convLines.add("algorithm,seed,evaluation,best_fitness");
        for (JsonNode r : out.path("runs")) {
            for (JsonNode p : r.path("convergence")) {
                convLines.add(ExperimentRunner.Csv.escape(r.path("algorithm").asText()) + "," + r.path("seed").asText()
                        + "," + p.get(0).asLong() + "," + num(p.get(1)));
            }
        }
        ExperimentRunner.Csv.write(dir.resolve("convergence.csv"), convLines);
        return dir;
    }

    private static String num(JsonNode node) {
        if (node == null || node.isNull() || node.isMissingNode() || !Double.isFinite(node.asDouble())) {
            return "";
        }
        return String.format(Locale.ROOT, "%.6g", node.asDouble());
    }
}
