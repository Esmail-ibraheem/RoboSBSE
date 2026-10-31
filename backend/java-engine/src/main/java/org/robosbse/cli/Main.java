package org.robosbse.cli;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Candidate;
import org.robosbse.core.RunControl;
import org.robosbse.core.SearchContext;
import org.robosbse.experiments.BenchmarkRunner;
import org.robosbse.experiments.CoverageCampaign;
import org.robosbse.experiments.SuiteRunner;
import org.robosbse.experiments.ExperimentRunner;
import org.robosbse.experiments.ProblemInstance;
import org.robosbse.experiments.Problems;
import org.robosbse.protocol.Capabilities;
import org.robosbse.protocol.Json;
import org.robosbse.protocol.ProtocolServer;
import org.robosbse.simulation.MapGenerator;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.FailureRecord;
import org.robosbse.testing.Failures;

/** Command-line entry point: the engine runs headless or serves the UI protocol. */
public final class Main {

    private static final String USAGE = """
            RoboSBSE engine %s

            Usage:
              robosbse-engine serve                          JSON Lines protocol on stdin/stdout (used by the UI)
              robosbse-engine run <experiment.json> [--out DIR] [--seed N] [--eval-threads N]
                                                             run one experiment headless
              robosbse-engine benchmark <benchmark.json> [--out DIR] [--threads N]
                                                             compare algorithms over several seeds
              robosbse-engine coverage <campaign.json> [--out DIR] [--threads N]
                                                             coverage-guided test generation for an instrumented controller
              robosbse-engine suite <suite.json> [--out DIR] [--threads N]
                                                             run a list of benchmarks / campaigns, write one report
              robosbse-engine reproduce <failure.json>       replay a saved failure (exit code 0 = reproduced)
              robosbse-engine plan <problem.json|experiment.json>
                                                             A* / Dijkstra reference for a navigation problem
              robosbse-engine generate-map scatter|maze [--seed N] [--out FILE]
              robosbse-engine capabilities                   print algorithms, problems, controllers as JSON
              robosbse-engine version
            """.formatted(ExperimentRunner.ENGINE_VERSION);

    private Main() {
    }

    public static void main(String[] args) throws Exception {
        if (args.length == 0) {
            System.out.print(USAGE);
            System.exit(2);
        }
        List<String> rest = new ArrayList<>(List.of(args).subList(1, args.length));
        int code = switch (args[0]) {
            case "serve" -> {
                new ProtocolServer(System.in, System.out, Path.of("").toAbsolutePath()).serve();
                yield 0;
            }
            case "run" -> run(rest);
            case "benchmark" -> benchmark(rest);
            case "coverage" -> coverage(rest);
            case "suite" -> suite(rest);
            case "reproduce" -> reproduce(rest);
            case "plan" -> plan(rest);
            case "generate-map" -> generateMap(rest);
            case "capabilities" -> {
                System.out.println(Json.pretty(Capabilities.describe()));
                yield 0;
            }
            case "version", "--version" -> {
                System.out.println("robosbse-engine " + ExperimentRunner.ENGINE_VERSION + " (protocol "
                        + Capabilities.PROTOCOL_VERSION + ", Java " + System.getProperty("java.version") + ")");
                yield 0;
            }
            default -> {
                System.err.print(USAGE);
                yield 2;
            }
        };
        System.exit(code);
    }

    private static String option(List<String> args, String name, String fallback) {
        int i = args.indexOf(name);
        if (i >= 0 && i + 1 < args.size()) {
            String value = args.get(i + 1);
            args.remove(i + 1);
            args.remove(i);
            return value;
        }
        return fallback;
    }

    private static Path requireFile(List<String> args) {
        if (args.isEmpty()) {
            throw new IllegalArgumentException("missing file argument");
        }
        return Path.of(args.get(0)).toAbsolutePath();
    }

    private static int run(List<String> args) {
        String out = option(args, "--out", "experiments/results");
        String seed = option(args, "--seed", null);
        String evalThreads = option(args, "--eval-threads", null);
        Path file = requireFile(args);
        JsonNode config = Json.read(file);
        if (seed != null) {
            ((ObjectNode) config).put("seed", Long.parseLong(seed));
        }
        if (evalThreads != null) {
            ObjectNode opts = config.path("options").isObject() ? (ObjectNode) config.path("options")
                    : ((ObjectNode) config).putObject("options");
            opts.put("threads", Integer.parseInt(evalThreads));
        }
        long budget = config.path("budget").path("maxEvaluations").asLong(1000);
        System.err.printf(Locale.ROOT, "running %s (%s, budget %d, seed %d)%n", config.path("name").asText(),
                config.path("algorithm").path("type").asText(), budget, config.path("seed").asLong(1));
        ExperimentRunner.Listener listener = new ExperimentRunner.Listener() {
            private long last;

            @Override
            public void onEvaluation(SearchContext<?> context, Candidate<?> candidate) {
                long now = System.nanoTime();
                if (now - last > 1_000_000_000L) {
                    last = now;
                    System.err.printf(Locale.ROOT, "  %6d / %d evaluations   best %.4f%n", context.evaluations(),
                            budget, context.best().fitness());
                }
            }
        };
        ObjectNode result = ExperimentRunner.run(config, file.getParent(), new RunControl(), listener,
                ExperimentRunner.Options.full(Path.of(out)));
        JsonNode best = result.path("best");
        System.out.printf(Locale.ROOT, "stop reason   : %s%n", result.path("stopReason").asText());
        System.out.printf(Locale.ROOT, "evaluations   : %d (%d ms wall, %d ms CPU, %d evaluation thread(s))%n",
                result.path("evaluations").asLong(), result.path("elapsedMs").asLong(), result.path("cpuMs").asLong(),
                result.path("threads").asInt(1));
        System.out.printf(Locale.ROOT, "best fitness  : %.6f (found at evaluation %d)%n",
                best.path("fitness").asDouble(), best.path("evaluation").asLong());
        System.out.printf(Locale.ROOT, "success       : %s%n", result.path("success").asBoolean());
        System.out.printf(Locale.ROOT, "best metrics  : %s%n", best.path("metrics"));
        if (result.has("failures")) {
            System.out.printf(Locale.ROOT, "targets hit   : %d (first at evaluation %d)%n",
                    result.path("targets").path("count").asLong(), result.path("targets").path("firstEvaluation").asLong());
            for (JsonNode f : result.path("failures")) {
                System.out.printf(Locale.ROOT, "  failure: %s, outcome %s, %d obstacles after minimisation%n",
                        f.path("violated"), f.path("outcome").asText(), f.path("scenario").path("obstacles").size());
            }
        }
        printReference(result.path("reference"));
        if (result.has("pareto")) {
            System.out.printf(Locale.ROOT, "pareto front  : %d points, normalised hypervolume %.4f%n",
                    result.path("pareto").path("points").size(), result.path("pareto").path("hypervolume").asDouble(Double.NaN));
        }
        System.out.printf("output        : %s%n", result.path("outputDir").asText());
        return 0;
    }

    private static int benchmark(List<String> args) {
        String out = option(args, "--out", "experiments/benchmarks");
        String threads = option(args, "--threads", null);
        Path file = requireFile(args);
        JsonNode config = Json.read(file);
        if (threads != null) {
            ((ObjectNode) config).put("threads", Integer.parseInt(threads));
        }
        ObjectNode result = BenchmarkRunner.run(config, file.getParent(), new RunControl(),
                (done, total, run) -> System.err.printf(Locale.ROOT, "  [%d/%d] %-24s seed %-4d best %.4f%n", done,
                        total, run.path("algorithm").asText(), run.path("seed").asLong(),
                        run.path("bestFitness").asDouble()),
                Path.of(out));
        System.out.printf("%-30s %5s %12s %12s %12s %8s %12s %12s%n", "algorithm", "runs", "median", "mean", "best",
                "success", "validation", "hypervolume");
        for (JsonNode s : result.path("summary")) {
            System.out.printf(Locale.ROOT, "%-30s %5d %12.4f %12.4f %12.4f %7.0f%% %12s %12s%n", s.path("algorithm").asText(),
                    s.path("runs").asInt(), s.path("median").asDouble(), s.path("mean").asDouble(),
                    s.path("min").asDouble(), 100 * s.path("successRate").asDouble(),
                    s.has("validation") ? String.format(Locale.ROOT, "%.4f", s.path("validation").path("median").asDouble()) : "-",
                    s.has("hypervolume") ? String.format(Locale.ROOT, "%.4f", s.path("hypervolume").path("median").asDouble()) : "-");
        }
        System.out.println();
        System.out.printf("%-22s %-22s %10s %10s %7s %s%n", "A", "B", "p", "p(Holm)", "A12", "effect");
        for (JsonNode p : result.path("pairwise")) {
            System.out.printf(Locale.ROOT, "%-22s %-22s %10.4g %10.4g %7.3f %s%n", p.path("a").asText(),
                    p.path("b").asText(), p.path("pValue").asDouble(), p.path("pHolm").asDouble(),
                    p.path("a12").asDouble(), p.path("effect").asText());
        }
        System.out.println();
        printReference(result.path("reference"));
        System.out.printf("%noutput: %s%n", result.path("outputDir").asText());
        return 0;
    }

    private static int suite(List<String> args) {
        String out = option(args, "--out", "experiments/suites");
        String threads = option(args, "--threads", null);
        Path file = requireFile(args);
        JsonNode config = Json.read(file);
        if (threads != null) {
            ((ObjectNode) config).put("threads", Integer.parseInt(threads));
        }
        ObjectNode result = SuiteRunner.run(config, file.getParent(), new RunControl(),
                (done, total, name) -> System.err.printf(Locale.ROOT, "  [%d/%d] finished %s%n", done, total, name),
                Path.of(out));
        System.out.println(SuiteRunner.report(result));
        System.out.printf("output: %s%n", result.path("outputDir").asText());
        return 0;
    }

    private static int coverage(List<String> args) {
        String out = option(args, "--out", "experiments/coverage");
        String threads = option(args, "--threads", null);
        Path file = requireFile(args);
        JsonNode config = Json.read(file);
        if (threads != null) {
            ((ObjectNode) config).put("threads", Integer.parseInt(threads));
        }
        ObjectNode result = CoverageCampaign.run(config, file.getParent(), new RunControl(),
                (done, total, t) -> System.err.printf(Locale.ROOT, "  [%d/%d] %-28s %-10s %s%n", done, total,
                        t.path("label").asText(), t.path("status").asText(),
                        t.has("evaluations") ? t.path("evaluations").asText() + " evaluations" : ""),
                Path.of(out));
        System.out.printf("controller      : %s (%s)%n", result.path("controller").asText(),
                result.path("instrumentation").asText());
        JsonNode s = result.path("summary");
        System.out.printf(Locale.ROOT, "search-based    : %d / %d outcomes covered (%.1f%%), %d evaluations%n",
                s.path("covered").asInt(), s.path("outcomes").asInt(), 100 * s.path("coverage").asDouble(),
                s.path("evaluations").asLong());
        if (result.has("random")) {
            JsonNode r = result.path("random");
            System.out.printf(Locale.ROOT, "random testing  : %d / %d outcomes covered (%.1f%%), %d evaluations; missed %s%n",
                    r.path("covered").asInt(), s.path("outcomes").asInt(), 100 * r.path("coverage").asDouble(),
                    r.path("evaluations").asLong(), r.path("uncovered"));
        }
        for (JsonNode t : result.path("targets")) {
            if (t.path("status").asText().equals("uncovered")) {
                System.out.printf(Locale.ROOT, "uncovered       : %s (approach level %s, branch distance %s)%n",
                        t.path("label").asText(), t.path("approachLevel").asText("?"), t.path("branchDistance").asText("?"));
            }
        }
        System.out.printf("output          : %s%n", result.path("outputDir").asText());
        return 0;
    }

    /** Prints the conventional baseline of the problem (A*, greedy dispatching or untuned defaults). */
    private static void printReference(JsonNode reference) {
        if (reference.has("astar")) {
            JsonNode astar = reference.path("astar");
            System.out.printf(Locale.ROOT, "A* reference  : fitness %.4f, %d node expansions (not fitness evaluations)%n",
                    astar.path("fitness").asDouble(Double.NaN), astar.path("expansions").asLong());
        } else if (reference.has("greedy")) {
            System.out.printf(Locale.ROOT, "greedy dispatching reference: fitness %.4f%n",
                    reference.path("greedy").path("fitness").asDouble(Double.NaN));
        } else if (reference.has("defaults")) {
            System.out.printf(Locale.ROOT, "untuned defaults reference : fitness %.4f (validation %.4f)%n",
                    reference.path("defaults").path("fitness").asDouble(Double.NaN),
                    reference.path("defaults").path("validationFitness").asDouble(Double.NaN));
        }
    }

    private static int reproduce(List<String> args) {
        Path file = requireFile(args);
        FailureRecord record = Json.convert(Json.read(file), FailureRecord.class);
        Failures.Reproduction r = Failures.reproduce(record, false);
        System.out.printf("controller          : %s%n", record.controller());
        System.out.printf("recorded violations : %s%n", record.violated());
        System.out.printf("replayed violations : %s (outcome %s)%n", r.violated(), r.execution().result().outcome());
        if (r.referenceViolated() != null) {
            System.out.printf("correct controller  : %s%n",
                    r.referenceViolated().isEmpty() ? "no violation (differential check passed)" : r.referenceViolated());
        }
        System.out.println(r.reproduced() ? "REPRODUCED" : "NOT REPRODUCED");
        return r.reproduced() ? 0 : 1;
    }

    private static int plan(List<String> args) {
        Path file = requireFile(args);
        JsonNode node = Json.read(file);
        JsonNode problem = node.has("problem") ? node.get("problem") : node;
        try (ProblemInstance<?> instance = Problems.create(problem, file.getParent())) {
            System.out.println(Json.pretty(instance.reference()));
            return 0;
        }
    }

    private static int generateMap(List<String> args) {
        String seedText = option(args, "--seed", "1");
        String out = option(args, "--out", null);
        String kind = args.isEmpty() ? "scatter" : args.get(0);
        long seed = Long.parseLong(seedText);
        WorldMap map = kind.equals("maze")
                ? MapGenerator.maze(seed, 8, 8, 2.5, 0.2)
                : MapGenerator.scatter(seed, 20, 20, 15, 0.25);
        if (out == null) {
            System.out.println(Json.pretty(map));
        } else {
            Json.write(Path.of(out), map);
            System.out.println("wrote " + out);
        }
        return 0;
    }
}
