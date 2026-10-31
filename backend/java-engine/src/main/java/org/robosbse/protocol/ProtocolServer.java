package org.robosbse.protocol;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Candidate;
import org.robosbse.core.RunControl;
import org.robosbse.core.SearchContext;
import org.robosbse.experiments.BenchmarkRunner;
import org.robosbse.experiments.CoverageCampaign;
import org.robosbse.experiments.ExperimentRunner;
import org.robosbse.experiments.ProblemInstance;
import org.robosbse.experiments.Problems;
import org.robosbse.geometry.Vec2;
import org.robosbse.problems.NavigationProblem;
import org.robosbse.simulation.MapGenerator;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.FailureRecord;
import org.robosbse.testing.Failures;

/**
 * JSON Lines protocol between the UI and the engine (version {@value Capabilities#PROTOCOL_VERSION}).
 *
 * <p>Every line is one JSON object with {@code "v"} (protocol version) and {@code "type"}. Requests
 * carry an {@code "id"} chosen by the client; all responses and events for that request echo it.
 * Searches run on worker threads and stream {@code progress} and {@code best} events; the whole
 * search — including every fitness evaluation — stays inside this process. See
 * {@code docs/architecture/protocol.md} for the message catalogue.
 */
public final class ProtocolServer {

    private static final long PROGRESS_INTERVAL_NANOS = 100_000_000L;
    private static final long BEST_INTERVAL_NANOS = 150_000_000L;

    private final BufferedReader in;
    private final PrintStream out;
    private final Path baseDir;
    private final Map<String, RunControl> runs = new ConcurrentHashMap<>();
    private final ExecutorService workers = Executors.newCachedThreadPool(r -> {
        Thread t = new Thread(r, "robosbse-worker");
        t.setDaemon(true);
        return t;
    });

    public ProtocolServer(InputStream in, OutputStream out, Path baseDir) {
        this.in = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
        this.out = new PrintStream(out, false, StandardCharsets.UTF_8);
        this.baseDir = baseDir;
    }

    /** Serves until end of input or a shutdown request. */
    public void serve() throws IOException {
        ObjectNode ready = event("ready", null);
        ready.setAll(Capabilities.describe());
        send(ready);
        String line;
        while ((line = in.readLine()) != null) {
            if (line.isBlank()) {
                continue;
            }
            JsonNode msg;
            try {
                msg = Json.parse(line);
            } catch (IllegalArgumentException e) {
                send(error(null, "invalid_json", e.getMessage()));
                continue;
            }
            if (!handle(msg)) {
                break;
            }
        }
        runs.values().forEach(RunControl::cancel);
        workers.shutdown();
        try {
            workers.awaitTermination(2, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Handles one request; returns false on shutdown. */
    boolean handle(JsonNode msg) {
        String id = msg.hasNonNull("id") ? msg.get("id").asText() : null;
        int version = msg.path("v").asInt(Capabilities.PROTOCOL_VERSION);
        if (version != Capabilities.PROTOCOL_VERSION) {
            send(error(id, "unsupported_version", "engine speaks protocol " + Capabilities.PROTOCOL_VERSION
                    + ", request used " + version));
            return true;
        }
        String type = msg.path("type").asText("");
        try {
            switch (type) {
                case "hello" -> {
                    ObjectNode hello = event("hello", id);
                    hello.setAll(Capabilities.describe());
                    send(hello);
                }
                case "get_map" -> {
                    ObjectNode reply = event("map", id);
                    reply.set("map", Json.tree(MapLibrary.resolve(msg.get("map"), baseDir)));
                    send(reply);
                }
                case "generate_map" -> {
                    ObjectNode reply = event("map", id);
                    reply.set("map", Json.tree(generate(msg.path("params"))));
                    send(reply);
                }
                case "start" -> startRun(id, msg);
                case "benchmark" -> startBenchmark(id, msg);
                case "coverage" -> startCoverage(id, msg);
                case "pause", "resume", "cancel" -> control(id, type);
                case "simulate" -> async(id, () -> simulate(id, msg));
                case "plan" -> async(id, () -> plan(id, msg));
                case "reproduce" -> async(id, () -> reproduce(id, msg));
                case "shutdown" -> {
                    send(event("bye", id));
                    return false;
                }
                default -> send(error(id, "unknown_command", "unknown request type '" + type + "'"));
            }
        } catch (RuntimeException e) {
            send(error(id, "invalid_request", describe(e)));
        }
        return true;
    }

    private static WorldMap generate(JsonNode p) {
        String kind = p.path("kind").asText("scatter");
        long seed = p.path("seed").asLong(1);
        if (kind.equals("maze")) {
            return MapGenerator.maze(seed, p.path("cellsX").asInt(8), p.path("cellsY").asInt(8),
                    p.path("cellSize").asDouble(2.5), p.path("wall").asDouble(0.2));
        }
        return MapGenerator.scatter(seed, p.path("width").asDouble(20), p.path("height").asDouble(20),
                p.path("count").asInt(15), 0.25);
    }

    private void async(String id, Runnable task) {
        workers.submit(() -> {
            try {
                task.run();
            } catch (RuntimeException e) {
                send(error(id, "failed", describe(e)));
            }
        });
    }

    private void startRun(String id, JsonNode msg) {
        if (id == null) {
            throw new IllegalArgumentException("start requires an id");
        }
        if (runs.containsKey(id)) {
            throw new IllegalArgumentException("run id '" + id + "' is already active");
        }
        JsonNode experiment = msg.get("experiment");
        if (experiment == null || !experiment.isObject()) {
            throw new IllegalArgumentException("start requires an 'experiment' object");
        }
        Path output = msg.hasNonNull("output") ? Path.of(msg.get("output").asText()) : null;
        RunControl control = new RunControl();
        runs.put(id, control);
        long maxEvaluations = experiment.path("budget").path("maxEvaluations").asLong(1000);
        ObjectNode started = event("started", id);
        started.put("maxEvaluations", maxEvaluations);
        send(started);
        workers.submit(() -> {
            try {
                ObjectNode result = ExperimentRunner.run(experiment, baseDir, control,
                        new ProgressListener(id, maxEvaluations), ExperimentRunner.Options.full(output));
                ObjectNode finished = event("finished", id);
                finished.set("result", result);
                send(finished);
            } catch (RuntimeException e) {
                send(error(id, "run_failed", describe(e)));
            } finally {
                runs.remove(id);
            }
        });
    }

    private void startBenchmark(String id, JsonNode msg) {
        if (id == null) {
            throw new IllegalArgumentException("benchmark requires an id");
        }
        JsonNode benchmark = msg.get("benchmark");
        if (benchmark == null || !benchmark.isObject()) {
            throw new IllegalArgumentException("benchmark requires a 'benchmark' object");
        }
        Path output = msg.hasNonNull("output") ? Path.of(msg.get("output").asText()) : null;
        RunControl control = new RunControl();
        runs.put(id, control);
        send(event("started", id));
        workers.submit(() -> {
            try {
                ObjectNode result = BenchmarkRunner.run(benchmark, baseDir, control, (done, total, run) -> {
                    ObjectNode p = event("benchmark_progress", id);
                    p.put("finished", done);
                    p.put("total", total);
                    ObjectNode light = run.deepCopy();
                    light.remove("convergence");
                    light.remove("bestSolution");
                    p.set("run", light);
                    send(p);
                }, output);
                ObjectNode finished = event("benchmark_finished", id);
                finished.set("result", result);
                send(finished);
            } catch (RuntimeException e) {
                send(error(id, "benchmark_failed", describe(e)));
            } finally {
                runs.remove(id);
            }
        });
    }

    private void startCoverage(String id, JsonNode msg) {
        if (id == null) {
            throw new IllegalArgumentException("coverage requires an id");
        }
        JsonNode campaign = msg.get("campaign");
        if (campaign == null || !campaign.isObject()) {
            throw new IllegalArgumentException("coverage requires a 'campaign' object");
        }
        Path output = msg.hasNonNull("output") ? Path.of(msg.get("output").asText()) : null;
        RunControl control = new RunControl();
        runs.put(id, control);
        send(event("started", id));
        workers.submit(() -> {
            try {
                ObjectNode result = CoverageCampaign.run(campaign, baseDir, control, (done, total, target) -> {
                    ObjectNode p = event("coverage_progress", id);
                    p.put("finished", done);
                    p.put("total", total);
                    p.set("target", target);
                    send(p);
                }, output);
                ObjectNode finished = event("coverage_finished", id);
                finished.set("result", result);
                send(finished);
            } catch (RuntimeException e) {
                send(error(id, "coverage_failed", describe(e)));
            } finally {
                runs.remove(id);
            }
        });
    }

    private void control(String id, String action) {
        RunControl control = id == null ? null : runs.get(id);
        if (control == null) {
            send(error(id, "unknown_run", "no active run with id '" + id + "'"));
            return;
        }
        switch (action) {
            case "pause" -> control.pause();
            case "resume" -> control.resume();
            default -> control.cancel();
        }
        ObjectNode state = event("state", id);
        state.put("state", action.equals("pause") ? "paused" : action.equals("resume") ? "running" : "cancelling");
        send(state);
    }

    /**
     * Replays a solution for inspection. Any problem accepts {@code genome} (the array found in every
     * solution view); navigation additionally accepts an explicit {@code route} of waypoints. The
     * reply always carries {@code replay} in the same format as a result's replay.
     */
    private void simulate(String id, JsonNode msg) {
        try (ProblemInstance<?> instance = Problems.create(msg.path("problem"), baseDir)) {
            ObjectNode reply = event("simulation", id);
            if (msg.has("genome")) {
                simulateGenome(instance, msg.get("genome"), reply);
            } else if (instance.problem() instanceof NavigationProblem nav) {
                List<Vec2> route = new ArrayList<>();
                for (JsonNode p : msg.path("route")) {
                    route.add(new Vec2(p.get(0).asDouble(), p.get(1).asDouble()));
                }
                if (route.isEmpty() || !route.get(route.size() - 1).equals(nav.map().goal())) {
                    route.add(nav.map().goal());
                }
                var evaluation = nav.evaluateRoute(route);
                var episode = nav.replay(route);
                Json.putFinite(reply, "fitness", evaluation.fitness());
                reply.set("metrics", Json.tree(evaluation.metrics()));
                reply.put("outcome", episode.outcome().name());
                reply.set("trajectory", Json.tree(episode.trajectory()));
                ArrayNode r = reply.putArray("route");
                r.add(Json.MAPPER.createArrayNode().add(nav.map().start().x()).add(nav.map().start().y()));
                route.forEach(p -> r.add(Json.MAPPER.createArrayNode().add(p.x()).add(p.y())));
                ObjectNode replay = reply.putObject("replay");
                replay.put("outcome", episode.outcome().name());
                replay.set("metrics", Json.tree(evaluation.metrics()));
                replay.set("trajectory", Json.tree(episode.trajectory()));
                replay.set("route", r.deepCopy());
            } else {
                throw new IllegalArgumentException("simulate needs a 'genome' (or a 'route' for navigation)");
            }
            send(reply);
        }
    }

    private static <S> void simulateGenome(ProblemInstance<S> instance, JsonNode genome, ObjectNode reply) {
        S solution = instance.parse(genome);
        var evaluation = instance.problem().evaluate(instance.problem().repair(solution));
        Json.putFinite(reply, "fitness", evaluation.fitness());
        reply.set("metrics", Json.tree(evaluation.metrics()));
        reply.set("objectives", Json.tree(evaluation.objectives()));
        reply.set("solution", Json.tree(instance.problem().view(solution)));
        reply.set("replay", Json.tree(instance.replay(solution)));
    }

    private void plan(String id, JsonNode msg) {
        try (ProblemInstance<?> instance = Problems.create(msg.path("problem"), baseDir)) {
            ObjectNode reply = event("plan_result", id);
            Map<String, Object> reference = instance.reference();
            reply.set("reference", reference == null ? Json.object() : Json.tree(reference));
            send(reply);
        }
    }

    private void reproduce(String id, JsonNode msg) {
        FailureRecord record = Json.convert(msg.get("failure"), FailureRecord.class);
        Failures.Reproduction r = Failures.reproduce(record, true);
        ObjectNode reply = event("reproduction", id);
        reply.put("reproduced", r.reproduced());
        reply.set("expected", Json.tree(record.violated()));
        reply.set("violated", Json.tree(r.violated()));
        reply.set("referenceViolated", r.referenceViolated() == null ? null : Json.tree(r.referenceViolated()));
        reply.put("outcome", r.execution().result().outcome().name());
        reply.set("verdicts", Json.tree(r.execution().verdicts()));
        reply.set("trajectory", Json.tree(r.execution().result().trajectory()));
        send(reply);
    }

    /** Streams throttled progress and best-so-far snapshots of one run. */
    private final class ProgressListener implements ExperimentRunner.Listener {
        private final String id;
        private final long maxEvaluations;
        private long lastProgress;
        private long lastBest;
        private int sentConvergence;
        private final List<double[]> pendingConvergence = new ArrayList<>();
        private Object pendingBestView;
        private double pendingBestFitness;
        private Map<String, Double> pendingBestMetrics;

        ProgressListener(String id, long maxEvaluations) {
            this.id = id;
            this.maxEvaluations = maxEvaluations;
        }

        @Override
        public void onNewBest(SearchContext<?> context, Candidate<?> best, Object view) {
            pendingConvergence.add(new double[] {context.evaluations(), best.fitness()});
            pendingBestView = view;
            pendingBestFitness = best.fitness();
            pendingBestMetrics = best.evaluation().metrics();
        }

        @Override
        public void onEvaluation(SearchContext<?> context, Candidate<?> candidate) {
            long now = System.nanoTime();
            boolean last = context.evaluations() >= maxEvaluations;
            if (pendingBestView != null && (now - lastBest >= BEST_INTERVAL_NANOS || last)) {
                ObjectNode best = event("best", id);
                best.put("evaluation", context.evaluations());
                best.put("fitness", pendingBestFitness);
                best.set("metrics", Json.tree(pendingBestMetrics));
                best.set("solution", Json.tree(pendingBestView));
                send(best);
                pendingBestView = null;
                lastBest = now;
            }
            if (now - lastProgress >= PROGRESS_INTERVAL_NANOS || last) {
                ObjectNode p = event("progress", id);
                p.put("evaluations", context.evaluations());
                p.put("maxEvaluations", maxEvaluations);
                p.put("iteration", context.iteration());
                p.put("elapsedMs", context.elapsedMillis());
                Json.putFinite(p, "bestFitness", context.best() == null ? Double.NaN : context.best().fitness());
                Json.putFinite(p, "currentFitness", candidate.fitness());
                p.put("targets", context.targetCount());
                p.put("firstTarget", context.firstTargetEvaluation());
                p.set("stats", Json.tree(context.stats()));
                p.set("convergence", Json.tree(new ArrayList<>(pendingConvergence)));
                sentConvergence += pendingConvergence.size();
                pendingConvergence.clear();
                send(p);
                lastProgress = now;
            }
        }
    }

    private static ObjectNode event(String type, String id) {
        ObjectNode node = Json.object();
        node.put("v", Capabilities.PROTOCOL_VERSION);
        node.put("type", type);
        if (id != null) {
            node.put("id", id);
        }
        return node;
    }

    private static ObjectNode error(String id, String code, String message) {
        ObjectNode node = event("error", id);
        node.put("code", code);
        node.put("message", message);
        return node;
    }

    private static String describe(Throwable e) {
        String m = e.getMessage();
        return e.getClass().getSimpleName() + (m == null ? "" : ": " + m);
    }

    private void send(ObjectNode node) {
        String line = Json.compact(node);
        synchronized (out) {
            out.println(line);
            out.flush();
        }
    }
}
