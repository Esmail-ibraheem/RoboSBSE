package org.robosbse.experiments;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Candidate;
import org.robosbse.core.Params;
import org.robosbse.core.RunControl;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchObserver;
import org.robosbse.core.SearchResult;
import org.robosbse.protocol.Json;

/**
 * Executes one experiment configuration ({@code robosbse.experiment/1}) and produces a
 * self-contained result document. The configuration echoed in the result has its map inlined, so a
 * result file alone is enough to re-run the experiment.
 */
public final class ExperimentRunner {

    public static final String ENGINE_VERSION = "0.1.0";
    public static final String RESULT_SCHEMA = "robosbse.result/1";
    private static final DateTimeFormatter STAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss").withZone(ZoneOffset.UTC);

    /** Receives progress during a run (a typed view of {@link SearchObserver} for any problem). */
    public interface Listener {
        default void onEvaluation(SearchContext<?> context, Candidate<?> candidate) {
        }

        default void onNewBest(SearchContext<?> context, Candidate<?> best, Object view) {
        }

        default void onTarget(SearchContext<?> context, Candidate<?> target) {
        }
    }

    /** Per-run options. */
    public record Options(boolean replay, boolean reference, boolean describeTargets, Path outputDir) {
        public static Options full(Path outputDir) {
            return new Options(true, true, true, outputDir);
        }

        public static Options summaryOnly() {
            return new Options(false, false, false, null);
        }
    }

    private ExperimentRunner() {
    }

    /** Normalised copy of the configuration with defaults filled in and the map inlined. */
    public static ObjectNode normalise(JsonNode config, ProblemInstance<?> instance) {
        ObjectNode copy = config.deepCopy();
        if (!copy.has("schema")) {
            copy.put("schema", "robosbse.experiment/1");
        }
        if (!copy.has("name")) {
            copy.put("name", "experiment");
        }
        if (!copy.has("seed")) {
            copy.put("seed", 1);
        }
        ObjectNode problem = copy.get("problem") instanceof ObjectNode existing ? existing : copy.putObject("problem");
        instance.normalise(problem);
        return copy;
    }

    public static ObjectNode run(JsonNode config, Path baseDir, RunControl control, Listener listener, Options options) {
        try (ProblemInstance<?> instance = Problems.create(config.path("problem"), baseDir)) {
            return runTyped(instance, config, control, listener, options);
        }
    }

    private static <S> ObjectNode runTyped(ProblemInstance<S> instance, JsonNode rawConfig, RunControl control,
                                           Listener listener, Options options) {
        ObjectNode config = normalise(rawConfig, instance);
        String algorithmId = config.path("algorithm").path("type").asText();
        AlgorithmRegistry.Descriptor descriptor = AlgorithmRegistry.get(algorithmId);
        String representation = instance.problem().space().representation();
        boolean multiObjective = !instance.problem().objectiveNames().isEmpty();
        if (!descriptor.supports(representation, multiObjective)) {
            throw new IllegalArgumentException(descriptor.name() + " cannot run on " + instance.problem().type()
                    + " (representation '" + representation + "'" + (descriptor.multiObjective() && !multiObjective
                    ? ", no objective vector" : "") + "); supported: " + descriptor.representations());
        }
        Params algorithmParams = descriptor.withDefaults(Params.of(Json.toMap(config.path("algorithm").get("params"))));
        SearchAlgorithm algorithm = descriptor.create(algorithmParams);
        JsonNode budgetNode = config.path("budget");
        Budget budget = new Budget(budgetNode.path("maxEvaluations").asLong(1000), budgetNode.path("maxMillis").asLong(0));
        long seed = config.path("seed").asLong(1);
        JsonNode opts = config.path("options");

        SearchObserver<S> observer = new SearchObserver<>() {
            @Override
            public void onEvaluation(SearchContext<S> context, Candidate<S> candidate) {
                if (listener != null) {
                    listener.onEvaluation(context, candidate);
                }
            }

            @Override
            public void onNewBest(SearchContext<S> context, Candidate<S> best) {
                if (listener != null) {
                    listener.onNewBest(context, best, instance.view(best));
                }
            }

            @Override
            public void onTarget(SearchContext<S> context, Candidate<S> target) {
                if (listener != null) {
                    listener.onTarget(context, target);
                }
            }
        };
        SearchContext<S> context = SearchContext.builder(instance.problem(), budget, seed)
                .control(control)
                .observer(observer)
                .archiveSize(opts.path("archiveSize").asInt(10))
                .maxTargets(opts.path("maxTargets").asInt(200))
                .stopOnTarget(opts.path("stopOnTarget").asBoolean(false))
                .paretoCapacity(opts.path("paretoSize").asInt(100))
                .threads(Math.max(1, opts.path("threads").asInt(1)))
                .build();
        Instant started = Instant.now();
        SearchResult<S> result = SearchAlgorithm.execute(algorithm, context);

        ObjectNode out = Json.object();
        out.put("schema", RESULT_SCHEMA);
        out.put("engineVersion", ENGINE_VERSION);
        out.set("experiment", config);
        out.set("algorithmParams", Json.tree(algorithmParams.asMap()));
        out.put("startedAt", started.toString());
        out.put("finishedAt", Instant.now().toString());
        out.put("stopReason", result.stopReason().name());
        out.put("evaluations", result.evaluations());
        out.put("iterations", result.iterations());
        out.put("elapsedMs", result.elapsedMillis());
        out.put("cpuMs", result.cpuMillis());
        out.put("threads", result.threads());
        out.put("success", instance.success(result));
        out.set("best", candidateNode(instance, result.best()));
        if (result.best() != null) {
            Map<String, Object> extras = instance.bestExtras(result.best().solution());
            if (!extras.isEmpty()) {
                out.set("bestExtras", Json.tree(extras));
            }
        }
        instance.runExtras().forEach((key, value) -> out.set(key, Json.tree(value)));
        out.set("convergence", Json.tree(result.convergence()));
        out.set("series", Json.tree(result.series()));
        out.set("stats", Json.tree(result.stats()));
        ArrayNode archive = out.putArray("archive");
        for (Candidate<S> c : result.archive()) {
            archive.add(candidateNode(instance, c));
        }
        if (!instance.problem().objectiveNames().isEmpty()) {
            ObjectNode pareto = out.putObject("pareto");
            pareto.set("objectives", Json.tree(instance.problem().objectiveNames()));
            double[] reference = instance.problem().referencePoint();
            pareto.set("referencePoint", reference == null ? null : Json.tree(reference));
            Json.putFinite(pareto, "hypervolume", result.hypervolume());
            ArrayNode points = pareto.putArray("points");
            for (Candidate<S> c : result.pareto()) {
                ObjectNode p = candidateNode(instance, c);
                p.set("objectives", Json.tree(c.evaluation().objectives()));
                points.add(p);
            }
        }
        ObjectNode targets = out.putObject("targets");
        targets.put("count", result.targetCount());
        targets.put("firstEvaluation", result.firstTargetEvaluation());
        if (options.replay() && result.best() != null) {
            out.set("replay", Json.tree(instance.replay(result.best().solution())));
        }
        if (options.reference()) {
            Map<String, Object> reference = instance.reference();
            if (reference != null) {
                out.set("reference", Json.tree(reference));
            }
        }
        List<Object> failures = List.of();
        if (options.describeTargets() && result.targetCount() > 0) {
            Map<String, Object> provenance = new LinkedHashMap<>();
            provenance.put("experiment", config.path("name").asText());
            provenance.put("algorithm", algorithmId);
            provenance.put("seed", seed);
            failures = instance.describeTargets(result, provenance,
                    opts.path("minimizeFailures").asBoolean(true), opts.path("maxFailureRecords").asInt(10));
            out.set(instance.targetsField(), Json.tree(failures));
        }
        if (options.outputDir() != null) {
            Path dir = writeOutputs(options.outputDir(), config, out, result, failures, instance.targetsField(), algorithmId, seed);
            out.put("outputDir", dir.toAbsolutePath().toString());
        }
        return out;
    }

    private static <S> ObjectNode candidateNode(ProblemInstance<S> instance, Candidate<S> c) {
        if (c == null) {
            return null;
        }
        ObjectNode node = Json.object();
        node.put("fitness", c.fitness());
        node.put("feasible", c.evaluation().feasible());
        node.put("constraintViolation", c.evaluation().constraintViolation());
        node.put("evaluation", c.index());
        node.set("metrics", Json.tree(c.evaluation().metrics()));
        node.set("solution", Json.tree(instance.view(c)));
        return node;
    }

    private static <S> Path writeOutputs(Path root, ObjectNode config, ObjectNode result, SearchResult<S> search,
                                         List<Object> failures, String targetsField, String algorithmId, long seed) {
        String name = sanitize(config.path("name").asText("experiment"));
        Path dir = root.resolve(STAMP.format(Instant.now()) + "-" + name + "-" + algorithmId + "-s" + seed);
        int suffix = 1;
        while (Files.exists(dir)) {
            dir = root.resolve(STAMP.format(Instant.now()) + "-" + name + "-" + algorithmId + "-s" + seed + "-" + suffix++);
        }
        Json.write(dir.resolve("config.json"), config);
        Json.write(dir.resolve("result.json"), result);
        List<String> lines = new ArrayList<>();
        lines.add("evaluation,best_fitness");
        for (double[] p : search.convergence()) {
            lines.add((long) p[0] + "," + p[1]);
        }
        Csv.write(dir.resolve("convergence.csv"), lines);
        for (int i = 0; i < failures.size(); i++) {
            String item = targetsField.endsWith("s") ? targetsField.substring(0, targetsField.length() - 1) : targetsField;
            Json.write(dir.resolve(targetsField).resolve(item + "-" + (i + 1) + ".json"), failures.get(i));
        }
        return dir;
    }

    static String sanitize(String s) {
        String cleaned = s.replaceAll("[^A-Za-z0-9._-]+", "-");
        return cleaned.isEmpty() ? "experiment" : cleaned;
    }

    /** Small CSV helper. */
    public static final class Csv {
        private Csv() {
        }

        public static void write(Path path, List<String> lines) {
            try {
                if (path.getParent() != null) {
                    Files.createDirectories(path.getParent());
                }
                Files.write(path, lines);
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }

        public static String escape(String value) {
            if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
                return "\"" + value.replace("\"", "\"\"") + "\"";
            }
            return value;
        }
    }
}
