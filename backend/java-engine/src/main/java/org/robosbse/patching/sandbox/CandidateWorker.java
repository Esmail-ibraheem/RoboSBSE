package org.robosbse.patching.sandbox;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.URI;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.SimpleJavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import javax.tools.ToolProvider;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Params;
import org.robosbse.experiments.Problems;
import org.robosbse.patching.WorkCounter;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.Outcome;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerSetup;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;
import org.robosbse.testing.Verdict;

/**
 * Child-process entry point that compiles and tests candidate controllers. It is started by
 * {@link WorkerPool} in a separate JVM (heap limit, one CPU hint, killed on timeout), so a candidate that
 * loops forever, exhausts memory, crashes or calls {@code System.exit} cannot affect the engine.
 *
 * <p>Protocol (JSON lines on stdin/stdout): first a {@code setup} message with the test scenarios and
 * performance episodes (answered by {@code ready}), then any number of {@code evaluate} messages with a
 * class name and source text. Each candidate is compiled with {@code javax.tools} into a fresh directory
 * and loaded by a fresh class loader. Candidates' own output is discarded so it cannot corrupt the protocol.
 */
public final class CandidateWorker {

    private final PrintStream protocol;
    private final JavaCompiler compiler;
    private final StandardJavaFileManager files;
    private ControllerTestingProblem testing;
    private RobotSpec robot;
    private final List<Test> tests = new ArrayList<>();
    private final List<Episode> episodes = new ArrayList<>();

    private record Test(String name, Scenario scenario, boolean requireGoal) {
    }

    private record Episode(String name, WorldMap map, World world, int maxSteps) {
    }

    private CandidateWorker(PrintStream protocol) {
        this.protocol = protocol;
        this.compiler = ToolProvider.getSystemJavaCompiler();
        this.files = compiler == null ? null : compiler.getStandardFileManager(null, null, StandardCharsets.UTF_8);
    }

    public static void main(String[] args) throws IOException {
        PrintStream protocol = new PrintStream(new java.io.FileOutputStream(java.io.FileDescriptor.out), true,
                StandardCharsets.UTF_8);
        System.setOut(new PrintStream(OutputStream.nullOutputStream()));
        CandidateWorker worker = new CandidateWorker(protocol);
        BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
        String line;
        while ((line = in.readLine()) != null) {
            if (line.isBlank()) {
                continue;
            }
            JsonNode msg = Json.parse(line);
            ObjectNode reply = switch (msg.path("type").asText()) {
                case "setup" -> worker.setup(msg);
                case "evaluate" -> worker.evaluate(msg);
                default -> error("unknown message type");
            };
            reply.put("id", msg.path("id").asText(""));
            protocol.println(Json.MAPPER.writeValueAsString(reply));
        }
    }

    private static ObjectNode error(String message) {
        ObjectNode out = Json.object();
        out.put("type", "error");
        out.put("error", message);
        return out;
    }

    private ObjectNode setup(JsonNode msg) {
        if (compiler == null) {
            return error("no Java compiler available: the engine must run on a JDK, not a JRE");
        }
        WorldMap arena = Json.convert(msg.get("arena"), WorldMap.class);
        robot = Problems.robot(msg.get("robot"));
        testing = new ControllerTestingProblem(arena, Params.of(Json.toMap(msg.get("testing"))), Params.empty(), robot);
        tests.clear();
        for (JsonNode t : msg.path("tests")) {
            tests.add(new Test(t.path("name").asText(), Json.convert(t.get("scenario"), Scenario.class),
                    t.path("requireGoal").asBoolean(true)));
        }
        episodes.clear();
        for (JsonNode e : msg.path("episodes")) {
            WorldMap map = Json.convert(e.get("map"), WorldMap.class);
            episodes.add(new Episode(e.path("name").asText(map.name()), map, new World(map), e.path("maxSteps").asInt(2400)));
        }
        ObjectNode out = Json.object();
        out.put("type", "ready");
        out.put("tests", tests.size());
        out.put("episodes", episodes.size());
        return out;
    }

    private ObjectNode evaluate(JsonNode msg) {
        ObjectNode out = Json.object();
        out.put("type", "result");
        String className = msg.path("className").asText();
        long t0 = System.nanoTime();
        Path dir = null;
        try {
            dir = Files.createTempDirectory("robosbse-candidate");
            String errors = compile(className, msg.path("source").asText(), dir);
            out.put("compileMs", (System.nanoTime() - t0) / 1_000_000);
            if (errors != null) {
                out.put("compiled", false);
                out.put("error", errors);
                return out;
            }
            try (URLClassLoader loader = new URLClassLoader(new URL[] {dir.toUri().toURL()},
                    CandidateWorker.class.getClassLoader())) {
                Class<?> type = loader.loadClass(className);
                if (!Controller.class.isAssignableFrom(type)) {
                    out.put("compiled", false);
                    out.put("error", className + " does not implement Controller");
                    return out;
                }
                out.put("compiled", true);
                long t1 = System.nanoTime();
                ArrayNode results = out.putArray("tests");
                for (Test t : tests) {
                    results.add(runTest(type, t));
                }
                if (msg.path("episodes").asBoolean(false)) {
                    ArrayNode eps = out.putArray("episodes");
                    for (Episode e : episodes) {
                        eps.add(runEpisode(type, e));
                    }
                }
                out.put("runMs", (System.nanoTime() - t1) / 1_000_000);
            }
        } catch (Throwable e) {
            out.put("compiled", out.path("compiled").asBoolean(false));
            out.put("error", e.getClass().getSimpleName() + ": " + e.getMessage());
        } finally {
            delete(dir);
        }
        return out;
    }

    private String compile(String className, String source, Path dir) throws IOException {
        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        files.setLocation(StandardLocation.CLASS_OUTPUT, List.of(dir.toFile()));
        JavaFileObject unit = new SimpleJavaFileObject(URI.create("string:///" + className.replace('.', '/') + ".java"),
                JavaFileObject.Kind.SOURCE) {
            @Override
            public CharSequence getCharContent(boolean ignoreEncodingErrors) {
                return source;
            }
        };
        List<String> options = List.of("-classpath", System.getProperty("java.class.path"), "-proc:none", "-nowarn",
                "-g:none", "-implicit:none");
        boolean ok = compiler.getTask(null, files, diagnostics, options, null, List.of(unit)).call();
        if (ok) {
            return null;
        }
        StringBuilder sb = new StringBuilder();
        for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
            if (d.getKind() == Diagnostic.Kind.ERROR && sb.length() < 600) {
                sb.append("line ").append(d.getLineNumber()).append(": ").append(d.getMessage(null)).append('\n');
            }
        }
        return sb.toString().trim();
    }

    private static Controller instantiate(Class<?> type) throws ReflectiveOperationException {
        return (Controller) type.getDeclaredConstructor().newInstance();
    }

    private ObjectNode runTest(Class<?> type, Test t) {
        ObjectNode r = Json.object();
        r.put("name", t.name());
        try {
            ControllerTestingProblem.Execution ex = testing.execute(t.scenario(), "candidate", instantiate(type), false);
            List<String> violated = ex.violated();
            boolean reached = ex.result().outcome() == Outcome.REACHED_GOAL;
            boolean passed = violated.isEmpty() && (!t.requireGoal() || reached);
            double severity = 0;
            for (Verdict v : ex.verdicts()) {
                if (v.violated()) {
                    severity = Math.max(severity, Math.min(1, Math.max(0, -v.robustness())));
                }
            }
            if (t.requireGoal() && !reached) {
                double initial = t.scenario().start().position().distanceTo(t.scenario().goal());
                severity = Math.max(severity, Math.min(1, ex.result().finalDistanceToGoal() / Math.max(1e-6, initial)));
            }
            r.put("passed", passed);
            r.put("loss", passed ? 0 : 1 + severity);
            r.set("violated", Json.tree(violated));
            r.put("outcome", ex.result().outcome().name());
            r.put("time", ex.result().time());
        } catch (Throwable e) {
            r.put("passed", false);
            r.put("loss", 2.0);
            r.put("outcome", "ERROR");
            r.put("error", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
        return r;
    }

    private ObjectNode runEpisode(Class<?> type, Episode e) {
        ObjectNode r = Json.object();
        r.put("name", e.name());
        try {
            Controller c = instantiate(type);
            EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps(e.maxSteps()).withAutoComplete(true);
            ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), e.map().goal(), e.map().goalTolerance(),
                    List.of(e.map().goal()), Params.empty());
            long t0 = System.nanoTime();
            EpisodeResult res = Simulator.run(e.world(), robot, e.map().start(), e.map().goal(), e.map().goalTolerance(),
                    c, setup, cfg);
            long nanos = System.nanoTime() - t0;
            r.put("success", res.success());
            r.put("outcome", res.outcome().name());
            r.put("time", res.time());
            r.put("energy", res.energy());
            r.put("steps", res.steps());
            r.put("work", c instanceof WorkCounter w ? w.work() : -1);
            r.put("nanosPerStep", res.steps() == 0 ? 0 : nanos / res.steps());
        } catch (Throwable ex) {
            r.put("success", false);
            r.put("outcome", "ERROR");
            r.put("error", ex.getClass().getSimpleName() + ": " + ex.getMessage());
        }
        return r;
    }

    private static void delete(Path dir) {
        if (dir == null) {
            return;
        }
        try (Stream<Path> walk = Files.walk(dir)) {
            walk.sorted(Comparator.reverseOrder()).forEach(p -> p.toFile().delete());
        } catch (IOException ignored) {
            // Temporary files; the OS cleans the temp directory eventually.
        }
    }
}
