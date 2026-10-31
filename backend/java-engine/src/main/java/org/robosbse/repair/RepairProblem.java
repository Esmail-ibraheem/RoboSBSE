package org.robosbse.repair;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.DeltaDebugging;
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
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;

/**
 * Automated program repair of a faulty Java controller (generate-and-validate, in the style of GenProg).
 *
 * <p><b>Tests.</b> Scenarios come from the subject's saved failure records and from a seeded random
 * sample of the testing arena. A scenario is a usable test only if the trusted reference controller
 * violates no property in it and the subject's reference fix passes it. <em>Failing tests</em> are those
 * the faulty subject fails; <em>passing tests</em> are those it already passes (the regression suite).
 * A second, disjoint sample gives <em>held-out tests</em> that the search never sees.
 *
 * <p><b>Fitness (minimised).</b> {@code weightNegative * sum(loss of failing tests) + sum(loss of
 * passing tests) + sizePenalty * edits}, where a test's loss is 0 when it passes and 1 + severity
 * (depth of the worst property violation, or the distance left to the goal) when it fails. Candidates
 * that do not compile, time out or crash lose 2 per test. A patch is <b>plausible</b> (a target hit) when
 * it passes every search test. Passing a test suite does not prove a patch correct; plausible patches are
 * therefore validated on the held-out tests and compared with the reference fix.
 */
public final class RepairProblem implements Problem<Patch>, AutoCloseable {

    public static final String TYPE = "program-repair";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("subject", "rb1-stopping-distance",
                    "Faulty controller: a built-in subject id (" + String.join(", ", Subject.BUILTIN.subList(0, 5))
                            + ") or a robosbse.repair-subject/1 file."),
            ParamSpec.integer("maxEdits", 3, 1, 20, "Largest number of edits in a candidate patch."),
            ParamSpec.integer("failingTests", 5, 1, 100, "Failing tests used by the search."),
            ParamSpec.integer("passingTests", 15, 0, 1000, "Passing (regression) tests used by the search."),
            ParamSpec.integer("heldOutFailing", 5, 0, 100, "Held-out tests the faulty subject fails."),
            ParamSpec.integer("heldOutPassing", 30, 0, 1000, "Held-out tests the faulty subject passes."),
            ParamSpec.integer("poolSize", 200, 1, 100000, "Random scenarios sampled per test pool."),
            ParamSpec.integer("testSeed", 1, 0, Integer.MAX_VALUE, "Seed of the search-test pool."),
            ParamSpec.integer("heldOutSeed", 2, 0, Integer.MAX_VALUE, "Seed of the held-out pool."),
            ParamSpec.text("properties", "safety", "Properties every test checks ('safety', 'all' or a list)."),
            ParamSpec.integer("maxSteps", 1200, 10, 1_000_000, "Episode step limit of every test (dt = 0.05 s)."),
            ParamSpec.real("weightNegative", 2.0, 0, 1000, "Weight of the failing tests' loss."),
            ParamSpec.real("sizePenalty", 0.01, 0, 100, "Fitness penalty per edit (prefers small patches)."),
            ParamSpec.integer("workers", 1, 1, 64, "Candidate worker processes (parallel evaluation)."),
            ParamSpec.integer("timeoutMs", 10000, 100, 3_600_000, "Time limit per candidate before its worker is killed."));

    private final Subject subject;
    private final SourceCatalog catalog;
    private final PatchSpace space;
    private final WorldMap arena;
    private final Params testingParams;
    private final RobotSpec robot;
    private final List<Suites.TestCase> searchTests = new ArrayList<>();
    private final int failing;
    private final List<Suites.TestCase> heldOut = new ArrayList<>();
    private final int heldOutFailingCount;
    private final JsonNode fixedHeldOut;
    private final JsonNode originalSearch;
    private final double weightNegative;
    private final double sizePenalty;
    private final int workers;
    private final long timeoutMs;
    private final WorkerPool pool;
    private WorkerPool heldOutPool;

    public RepairProblem(Subject subject, Params params, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid program-repair parameters: " + String.join("; ", errors));
        }
        this.subject = subject;
        this.robot = robot;
        this.catalog = new SourceCatalog(subject.source(), subject.scope());
        this.space = new PatchSpace(catalog, params.getInt("maxEdits", 3));
        this.arena = MapLibrary.builtin("testing-arena");
        this.testingParams = Suites.testingParams(params.getString("properties", "safety"), params.getInt("maxSteps", 1200));
        this.weightNegative = params.getDouble("weightNegative", 2.0);
        this.sizePenalty = params.getDouble("sizePenalty", 0.01);
        this.workers = params.getInt("workers", 1);
        this.timeoutMs = params.getInt("timeoutMs", 10000);
        ControllerTestingProblem testing = new ControllerTestingProblem(arena, testingParams, Params.empty(), robot);

        int poolSize = params.getInt("poolSize", 200);
        List<Scenario> searchPool = new ArrayList<>(subject.failureScenarios());
        searchPool.addAll(Suites.randomScenarios(testing, params.getInt("testSeed", 1), poolSize));
        List<Suites.TestCase> searchCandidates = Suites.referenceCases(testing, searchPool, "t");
        List<Scenario> heldPool = new ArrayList<>(subject.heldOutScenarios());
        heldPool.addAll(Suites.randomScenarios(testing, params.getInt("heldOutSeed", 2), poolSize));
        List<Suites.TestCase> heldCandidates = Suites.referenceCases(testing, heldPool, "h");
        List<Suites.TestCase> all = new ArrayList<>(searchCandidates);
        all.addAll(heldCandidates);
        JsonNode original;
        JsonNode fixed;
        try (WorkerPool classify = new WorkerPool(Suites.setup(arena, testingParams, robot, all, List.of()), 1, timeoutMs * 10)) {
            original = classify.evaluate(subject.className(), subject.source(), false);
            fixed = subject.fixedSource() == null ? null : classify.evaluate(subject.className(), subject.fixedSource(), false);
        }
        if (!Suites.ran(original)) {
            throw new IllegalArgumentException("the unpatched subject does not run: " + Suites.status(original) + " "
                    + original.path("error").asText(""));
        }
        // Indices into 'all' of the failing / passing tests of both pools.
        List<Integer> neg = new ArrayList<>();
        List<Integer> pos = new ArrayList<>();
        List<Integer> heldNeg = new ArrayList<>();
        List<Integer> heldPos = new ArrayList<>();
        for (int i = 0; i < all.size(); i++) {
            if (fixed != null && !Suites.passed(fixed, i)) {
                continue; // the reference fix must pass every test
            }
            boolean subjectFails = !Suites.passed(original, i);
            if (i < searchCandidates.size()) {
                (subjectFails ? neg : pos).add(i);
            } else {
                (subjectFails ? heldNeg : heldPos).add(i);
            }
        }
        if (neg.isEmpty()) {
            throw new IllegalArgumentException("subject " + subject.id() + " fails none of the generated tests; nothing to repair");
        }
        List<Integer> searchIndex = new ArrayList<>(neg.subList(0, Math.min(neg.size(), params.getInt("failingTests", 5))));
        this.failing = searchIndex.size();
        searchIndex.addAll(pos.subList(0, Math.min(pos.size(), params.getInt("passingTests", 15))));
        List<Integer> heldIndex = new ArrayList<>(heldNeg.subList(0, Math.min(heldNeg.size(), params.getInt("heldOutFailing", 5))));
        this.heldOutFailingCount = heldIndex.size();
        heldIndex.addAll(heldPos.subList(0, Math.min(heldPos.size(), params.getInt("heldOutPassing", 30))));
        for (int i : searchIndex) {
            searchTests.add(all.get(i));
        }
        // Results of the reference fix on the held-out tests, for the equivalence check.
        ArrayNode fixedTests = Json.MAPPER.createArrayNode();
        for (int i : heldIndex) {
            heldOut.add(all.get(i));
            fixedTests.add(fixed == null ? Json.object() : fixed.path("tests").path(i));
        }
        this.fixedHeldOut = fixed == null ? null : fixedTests;
        this.pool = new WorkerPool(Suites.setup(arena, testingParams, robot, searchTests, List.of()), workers, timeoutMs);
        this.originalSearch = pool.evaluate(subject.className(), subject.source(), false);
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

    public List<Suites.TestCase> searchTests() {
        return searchTests;
    }

    public int failingTests() {
        return failing;
    }

    public List<Suites.TestCase> heldOutTests() {
        return heldOut;
    }

    public int heldOutFailing() {
        return heldOutFailingCount;
    }

    public WorkerPool pool() {
        return pool;
    }

    /** Worker result of a patch on the search tests (cached by source). */
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
        return pool.evaluate(subject.className(), source, false);
    }

    @Override
    public Evaluation evaluate(Patch patch) {
        return evaluation(patch, run(patch));
    }

    Evaluation evaluation(Patch patch, JsonNode result) {
        double negLoss = 0;
        double posLoss = 0;
        int failedNeg = 0;
        int failedPos = 0;
        for (int i = 0; i < searchTests.size(); i++) {
            double loss = Suites.loss(result, i);
            boolean passed = Suites.passed(result, i);
            if (i < failing) {
                negLoss += loss;
                failedNeg += passed ? 0 : 1;
            } else {
                posLoss += loss;
                failedPos += passed ? 0 : 1;
            }
        }
        boolean ran = Suites.ran(result);
        double fitness = weightNegative * negLoss + posLoss + sizePenalty * patch.size();
        if (!result.path("compiled").asBoolean(false)) {
            fitness += 1; // worse than a compiling candidate that fails every test
        }
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("failedTests", (double) (failedNeg + failedPos));
        metrics.put("failedFailingTests", (double) failedNeg);
        metrics.put("failedPassingTests", (double) failedPos);
        metrics.put("compiled", result.path("compiled").asBoolean(false) ? 1.0 : 0.0);
        metrics.put("ran", ran ? 1.0 : 0.0);
        metrics.put("edits", (double) patch.size());
        double[] behaviour = {
                failing == 0 ? 0 : (double) (failing - failedNeg) / failing,
                searchTests.size() == failing ? 0 : (double) (searchTests.size() - failing - failedPos) / (searchTests.size() - failing)};
        return new Evaluation(fitness, failedNeg + failedPos, metrics, null, behaviour);
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        return evaluation.metric("ran", 0) > 0 && evaluation.metric("failedTests", 1) == 0;
    }

    /** Whether a patch passes every search test (uses the cache, not charged to any budget). */
    public boolean plausible(Patch patch) {
        return isTarget(evaluate(patch));
    }

    /** ddmin over the edits of a plausible patch: a 1-minimal plausible sub-patch. */
    public Patch minimise(Patch patch) {
        List<Patch.Edit> edits = DeltaDebugging.ddmin(patch.edits(), subset -> !subset.isEmpty() && plausible(Patch.of(subset)));
        return edits.isEmpty() ? patch : Patch.of(edits);
    }

    /** Worker result of a patch on the held-out tests. */
    public synchronized JsonNode runHeldOut(Patch patch) {
        if (heldOutPool == null) {
            heldOutPool = new WorkerPool(Suites.setup(arena, testingParams, robot, heldOut, List.of()), 1, timeoutMs);
        }
        return heldOutPool.evaluate(subject.className(), catalog.apply(patch), false);
    }

    /**
     * Held-out validation of a patch: tests passed, and whether it behaves exactly like the reference fix
     * (same verdict, outcome and episode time on every held-out test).
     */
    public Map<String, Object> validate(Patch patch) {
        JsonNode r = runHeldOut(patch);
        int passed = 0;
        int passedFailing = 0;
        boolean equivalent = fixedHeldOut != null && Suites.ran(r);
        for (int i = 0; i < heldOut.size(); i++) {
            boolean ok = Suites.passed(r, i);
            passed += ok ? 1 : 0;
            if (i < heldOutFailingCount && ok) {
                passedFailing++;
            }
            if (equivalent) {
                JsonNode a = r.path("tests").path(i);
                JsonNode b = fixedHeldOut.path(i);
                equivalent = a.path("passed").asBoolean() == b.path("passed").asBoolean()
                        && a.path("outcome").asText().equals(b.path("outcome").asText())
                        && a.path("time").asDouble(Double.NaN) == b.path("time").asDouble(Double.NaN);
            }
        }
        String patched = catalog.apply(patch);
        String fix = subject.fixedSource();
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("heldOutTests", heldOut.size());
        out.put("heldOutPassed", passed);
        out.put("heldOutFailingFixed", passedFailing);
        out.put("heldOutFailing", heldOutFailingCount);
        out.put("passesHeldOut", passed == heldOut.size());
        out.put("equivalentToReferenceFix", equivalent);
        out.put("identicalToReferenceFix", fix != null && SourceCatalog.normalise(patched).equals(SourceCatalog.normalise(fix)));
        out.put("status", Suites.status(r));
        return out;
    }

    /** The unpatched subject's results on the search tests. */
    public JsonNode originalResults() {
        return originalSearch;
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
