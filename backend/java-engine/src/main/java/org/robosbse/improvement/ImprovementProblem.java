package org.robosbse.improvement;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.patching.Patch;
import org.robosbse.patching.PatchSpace;
import org.robosbse.patching.SourceCatalog;
import org.robosbse.patching.Subject;
import org.robosbse.patching.Suites;
import org.robosbse.patching.sandbox.WorkerPool;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.Outcome;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;

/**
 * Genetic improvement of a working controller: search for small source edits (the restricted
 * operators of {@link org.robosbse.patching.MutationPoint}, limited to the subject's scope) that make
 * it faster or cheaper to run without breaking it.
 *
 * <p><b>Correctness suite.</b> Scenarios from the testing arena that the unmodified baseline passes; a
 * candidate must pass all of them (no property violated, goal reached wherever the baseline reaches
 * it), and must still complete every performance map the baseline completes. Held-out scenarios
 * validate the final result.
 *
 * <p><b>Performance</b> is measured on performance maps relative to the baseline: {@code timeRatio}
 * (mean travel time) and {@code workRatio} (counted work units per control step, a deterministic
 * proxy for computational cost; wall-clock time per step is reported as well but not used).
 * Fitness of a correct candidate = {@code timeRatio + workWeight * workRatio + sizePenalty * edits}
 * (the baseline scores {@code 1 + workWeight}); incorrect candidates score {@code 10 + failures}.
 * Objectives for NSGA-II: (timeRatio, workRatio). A target hit is a correct strict improvement.
 */
public final class ImprovementProblem implements Problem<Patch>, AutoCloseable {

    public static final String TYPE = "genetic-improvement";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("subject", "gi-reactive-baseline", "Controller to improve (built-in id or robosbse.repair-subject/1 file)."),
            ParamSpec.text("maps", "obstacle-course,warehouse,empty-field", "Performance maps (start to goal)."),
            ParamSpec.integer("maxSteps", 2400, 10, 1_000_000, "Step limit of the performance episodes."),
            ParamSpec.integer("tests", 20, 1, 1000, "Correctness tests used by the search."),
            ParamSpec.integer("heldOutTests", 40, 0, 10000, "Held-out correctness tests for the final result."),
            ParamSpec.integer("poolSize", 120, 1, 100000, "Random scenarios sampled per test pool."),
            ParamSpec.integer("testSeed", 1, 0, Integer.MAX_VALUE, "Seed of the search-test pool."),
            ParamSpec.integer("heldOutSeed", 2, 0, Integer.MAX_VALUE, "Seed of the held-out pool."),
            ParamSpec.text("properties", "safety", "Properties every test checks."),
            ParamSpec.integer("testMaxSteps", 1200, 10, 1_000_000, "Step limit of every correctness test."),
            ParamSpec.real("workWeight", 1.0, 0, 100, "Weight of the work ratio against the time ratio."),
            ParamSpec.real("sizePenalty", 0.001, 0, 100, "Fitness penalty per edit."),
            ParamSpec.integer("maxEdits", 3, 1, 20, "Largest number of edits in a candidate."),
            ParamSpec.integer("workers", 1, 1, 64, "Candidate worker processes."),
            ParamSpec.integer("timeoutMs", 10000, 100, 3_600_000, "Time limit per candidate."));

    private final Subject subject;
    private final SourceCatalog catalog;
    private final PatchSpace space;
    private final WorldMap arena;
    private final Params testingParams;
    private final RobotSpec robot;
    private final List<Suites.TestCase> tests = new ArrayList<>();
    private final List<Suites.TestCase> heldOut = new ArrayList<>();
    private final List<Suites.Episode> episodes = new ArrayList<>();
    private final double workWeight;
    private final double sizePenalty;
    private final long timeoutMs;
    private final WorkerPool pool;
    private WorkerPool heldOutPool;
    private final JsonNode baseline;
    private final boolean[] baselineSuccess;
    private final double baseTime;
    private final double baseWork;

    public ImprovementProblem(Subject subject, List<WorldMap> maps, Params params, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid genetic-improvement parameters: " + String.join("; ", errors));
        }
        this.subject = subject;
        this.robot = robot;
        this.catalog = new SourceCatalog(subject.source(), subject.scope());
        this.space = new PatchSpace(catalog, params.getInt("maxEdits", 3));
        this.arena = MapLibrary.builtin("testing-arena");
        this.testingParams = Suites.testingParams(params.getString("properties", "safety"), params.getInt("testMaxSteps", 1200));
        this.workWeight = params.getDouble("workWeight", 1.0);
        this.sizePenalty = params.getDouble("sizePenalty", 0.001);
        this.timeoutMs = params.getInt("timeoutMs", 10000);
        for (WorldMap m : maps) {
            episodes.add(new Suites.Episode(m.name(), m, params.getInt("maxSteps", 2400)));
        }
        ControllerTestingProblem testing = new ControllerTestingProblem(arena, testingParams, Params.empty(), robot);
        int poolSize = params.getInt("poolSize", 120);
        List<Scenario> searchPool = Suites.randomScenarios(testing, params.getInt("testSeed", 1), poolSize);
        List<Scenario> heldPool = Suites.randomScenarios(testing, params.getInt("heldOutSeed", 2), poolSize);
        // Classify with goal not required, then require the goal wherever the baseline reaches it.
        List<Suites.TestCase> candidates = new ArrayList<>();
        for (int i = 0; i < searchPool.size() + heldPool.size(); i++) {
            Scenario s = i < searchPool.size() ? searchPool.get(i) : heldPool.get(i - searchPool.size());
            candidates.add(new Suites.TestCase((i < searchPool.size() ? "t" : "h") + i, s, false));
        }
        JsonNode classified;
        try (WorkerPool classify = new WorkerPool(Suites.setup(arena, testingParams, robot, candidates, List.of()), 1, timeoutMs * 10)) {
            classified = classify.evaluate(subject.className(), subject.source(), false);
        }
        if (!Suites.ran(classified)) {
            throw new IllegalArgumentException("the baseline subject does not run: " + classified.path("error").asText(""));
        }
        int wantTests = params.getInt("tests", 20);
        int wantHeld = params.getInt("heldOutTests", 40);
        for (int i = 0; i < candidates.size(); i++) {
            JsonNode t = classified.path("tests").path(i);
            if (!t.path("passed").asBoolean(false)) {
                continue; // the baseline must pass every correctness test
            }
            boolean reached = Outcome.REACHED_GOAL.name().equals(t.path("outcome").asText());
            Suites.TestCase c = new Suites.TestCase(candidates.get(i).name(), candidates.get(i).scenario(), reached);
            if (i < searchPool.size()) {
                if (tests.size() < wantTests) {
                    tests.add(c);
                }
            } else if (heldOut.size() < wantHeld) {
                heldOut.add(c);
            }
        }
        this.pool = new WorkerPool(Suites.setup(arena, testingParams, robot, tests, episodes), params.getInt("workers", 1), timeoutMs);
        this.baseline = pool.evaluate(subject.className(), subject.source(), true);
        this.baselineSuccess = new boolean[episodes.size()];
        double t = 0;
        long work = 0;
        long steps = 0;
        int n = 0;
        for (int i = 0; i < episodes.size(); i++) {
            JsonNode e = baseline.path("episodes").path(i);
            baselineSuccess[i] = e.path("success").asBoolean(false);
            if (baselineSuccess[i]) {
                t += e.path("time").asDouble();
                n++;
            }
            work += Math.max(0, e.path("work").asLong(0));
            steps += e.path("steps").asLong(0);
        }
        if (n == 0) {
            throw new IllegalArgumentException("the baseline completes none of the performance maps");
        }
        this.baseTime = t / n;
        this.baseWork = steps == 0 ? 1 : Math.max(1e-9, (double) work / steps);
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public PatchSpace space() {
        return space;
    }

    public Subject subject() {
        return subject;
    }

    public SourceCatalog catalog() {
        return catalog;
    }

    public List<Suites.TestCase> tests() {
        return tests;
    }

    public List<Suites.TestCase> heldOutTests() {
        return heldOut;
    }

    public WorkerPool pool() {
        return pool;
    }

    public JsonNode run(Patch patch) {
        String source;
        try {
            source = catalog.apply(patch);
        } catch (RuntimeException e) {
            ObjectNode bad = Json.object();
            bad.put("compiled", false);
            bad.put("error", "patch could not be applied: " + e.getMessage());
            return bad;
        }
        return pool.evaluate(subject.className(), source, true);
    }

    @Override
    public Evaluation evaluate(Patch patch) {
        return evaluation(patch, run(patch));
    }

    /** Measurements of a worker result relative to the baseline. */
    public Map<String, Double> measure(JsonNode r) {
        int failedTests = 0;
        for (int i = 0; i < tests.size(); i++) {
            failedTests += Suites.passed(r, i) ? 0 : 1;
        }
        int failedEpisodes = 0;
        double time = 0;
        long work = 0;
        long steps = 0;
        double nanos = 0;
        int n = 0;
        for (int i = 0; i < episodes.size(); i++) {
            JsonNode e = r.path("episodes").path(i);
            boolean ok = e.path("success").asBoolean(false);
            if (baselineSuccess[i] && !ok) {
                failedEpisodes++;
            }
            if (baselineSuccess[i] && ok) {
                time += e.path("time").asDouble();
                n++;
            }
            work += Math.max(0, e.path("work").asLong(0));
            steps += e.path("steps").asLong(0);
            nanos += e.path("nanosPerStep").asDouble(0) / episodes.size();
        }
        boolean valid = Suites.ran(r) && failedTests == 0 && failedEpisodes == 0;
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("valid", valid ? 1.0 : 0.0);
        m.put("failedTests", (double) failedTests);
        m.put("failedEpisodes", (double) failedEpisodes);
        m.put("meanTime", n == 0 ? Double.NaN : time / n);
        m.put("workPerStep", steps == 0 ? Double.NaN : (double) work / steps);
        m.put("timeRatio", n == 0 ? Double.NaN : (time / n) / baseTime);
        m.put("workRatio", steps == 0 ? Double.NaN : ((double) work / steps) / baseWork);
        m.put("nanosPerStep", nanos);
        m.put("compiled", r.path("compiled").asBoolean(false) ? 1.0 : 0.0);
        return m;
    }

    Evaluation evaluation(Patch patch, JsonNode r) {
        Map<String, Double> m = measure(r);
        boolean valid = m.get("valid") > 0;
        double failures = m.get("failedTests") + m.get("failedEpisodes");
        double fitness;
        double[] objectives;
        if (valid) {
            fitness = m.get("timeRatio") + workWeight * m.get("workRatio") + sizePenalty * patch.size();
            objectives = new double[] {m.get("timeRatio"), m.get("workRatio")};
        } else {
            fitness = 10 + failures + (m.get("compiled") > 0 ? 0 : 1);
            objectives = new double[] {10, 10};
        }
        m.put("edits", (double) patch.size());
        m.values().removeIf(v -> !Double.isFinite(v));
        double[] behaviour = valid ? new double[] {Math.min(1, m.get("timeRatio") / 2), Math.min(1, m.get("workRatio") / 2)}
                : new double[] {1, 1};
        return new Evaluation(fitness, valid ? 0 : Math.max(1, failures), m, objectives, behaviour);
    }

    /** Fitness of the unchanged baseline. */
    public double baselineFitness() {
        return 1 + workWeight;
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        return evaluation.metric("valid", 0) > 0 && evaluation.fitness() < baselineFitness() - 1e-9;
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("timeRatio", "workRatio");
    }

    @Override
    public double[] referencePoint() {
        return new double[] {1.5, 1.5};
    }

    /** Baseline measurements (relative values 1 by definition). */
    public Map<String, Double> baselineMetrics() {
        return measure(baseline);
    }

    /** Held-out correctness of a patch. */
    public synchronized Map<String, Object> validate(Patch patch) {
        if (heldOutPool == null) {
            heldOutPool = new WorkerPool(Suites.setup(arena, testingParams, robot, heldOut, List.of()), 1, timeoutMs);
        }
        JsonNode r = heldOutPool.evaluate(subject.className(), catalog.apply(patch), false);
        int passed = 0;
        for (int i = 0; i < heldOut.size(); i++) {
            passed += Suites.passed(r, i) ? 1 : 0;
        }
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("heldOutTests", heldOut.size());
        out.put("heldOutPassed", passed);
        out.put("passesHeldOut", passed == heldOut.size());
        out.put("status", Suites.status(r));
        return out;
    }

    @Override
    public Object view(Patch patch) {
        Map<String, Object> view = new LinkedHashMap<>();
        view.put("patch", patch.toString());
        view.put("edits", catalog.describe(patch));
        view.put("size", patch.size());
        view.put("genome", patch.toString());
        return view;
    }

    @Override
    public void close() {
        pool.close();
        synchronized (this) {
            if (heldOutPool != null) {
                heldOutPool.close();
            }
        }
    }
}
