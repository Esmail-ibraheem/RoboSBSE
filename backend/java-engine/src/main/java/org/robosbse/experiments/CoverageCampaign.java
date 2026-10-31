package org.robosbse.experiments;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

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
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.protocol.Json;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.coverage.Branch;
import org.robosbse.testing.coverage.CoverageAccumulator;

/**
 * Coverage-guided test generation for an instrumented controller (schema {@code robosbse.coverage/1}).
 *
 * <p>Every outcome of every instrumented decision is a coverage target. Targets are visited in
 * declaration order; a target that earlier searches already covered by accident is skipped
 * ("collateral"). Otherwise a search with {@code fitness = branch:<decision>:<outcome>} (approach level
 * plus normalised branch distance) runs until it covers the target or exhausts its share of the
 * budget: the remaining budget divided equally among the targets still uncovered, so budget saved on
 * easy targets goes to the hard ones. All scenarios of all searches count towards the campaign's
 * coverage. A random-testing baseline with the same total budget is measured on the same instrumentation.
 *
 * <p>Coverage is measured, never estimated: only controllers that implement
 * {@link org.robosbse.testing.coverage.Instrumented} can run a campaign.
 */
public final class CoverageCampaign {

    public static final String SCHEMA = "robosbse.coverage/1";

    /** Progress callback after each target. */
    public interface Listener {
        void onTarget(int done, int total, ObjectNode target);
    }

    private CoverageCampaign() {
    }

    public static ObjectNode run(JsonNode campaign, Path baseDir, RunControl control, Listener listener, Path outputDir) {
        JsonNode problemNode = campaign.path("problem");
        if (!ControllerTestingProblem.TYPE.equals(problemNode.path("type").asText())) {
            throw new IllegalArgumentException("a coverage campaign needs a controller-testing problem");
        }
        ProblemInstance<?> baseInstance = Problems.create(problemNode, baseDir);
        ControllerTestingProblem base = (ControllerTestingProblem) baseInstance.problem();
        if (base.coverage() == null) {
            throw new IllegalArgumentException("controller " + base.controllerId()
                    + " has no coverage instrumentation; coverage is unavailable");
        }
        long total = campaign.path("budget").path("maxEvaluations").asLong(5000);
        long seed = campaign.path("seed").asLong(1);
        int threads = Math.max(1, campaign.path("threads").asInt(1));
        JsonNode algorithmNode = campaign.path("algorithm");
        String algorithmType = algorithmNode.path("type").asText("genetic-algorithm");
        JsonNode algorithmParams = algorithmNode.path("params").isObject() ? algorithmNode.path("params") : Json.object();
        AlgorithmRegistry.Descriptor descriptor = AlgorithmRegistry.get(algorithmType);
        if (!descriptor.supports("real-vector", false)) {
            throw new IllegalArgumentException(algorithmType + " cannot generate test scenarios");
        }
        Instant started = Instant.now();

        List<Branch> branches = base.branches();
        CoverageAccumulator coverage = new CoverageAccumulator(branches);
        int targetCount = 2 * branches.size();
        ArrayNode targets = Json.MAPPER.createArrayNode();
        long remaining = total;
        int index = 0;
        for (Branch branch : branches) {
            for (boolean outcome : new boolean[] {true, false}) {
                ObjectNode t = targets.addObject();
                t.put("decision", branch.id());
                t.put("outcome", outcome);
                t.put("label", Branch.outcome(branch.id(), outcome));
                t.put("description", branch.description());
                t.put("parent", branch.parent() == null ? null : Branch.outcome(branch.parent(), branch.parentOutcome()));
                if (control.isCancelled()) {
                    t.put("status", "not-run");
                } else if (coverage.covered(branch.id(), outcome)) {
                    t.put("status", "collateral");
                    t.put("evaluations", 0);
                } else if (remaining <= 0) {
                    t.put("status", "uncovered");
                    t.put("evaluations", 0);
                } else {
                    long share = Math.max(1, remaining / uncoveredFrom(coverage, branches, index));
                    long used = search(t, problemNode, baseDir, branch, outcome, share, seed + index, threads,
                            descriptor, algorithmParams, control, coverage);
                    remaining -= used;
                }
                index++;
                if (listener != null) {
                    listener.onTarget(index, targetCount, t);
                }
            }
        }
        // Targets whose own search failed may still have been covered by later searches.
        for (JsonNode node : targets) {
            ObjectNode t = (ObjectNode) node;
            boolean covered = coverage.covered(t.path("decision").asText(), t.path("outcome").asBoolean());
            if (covered && t.path("status").asText().equals("uncovered")) {
                t.put("status", "collateral");
            }
            double[] witness = coverage.witness(t.path("decision").asText(), t.path("outcome").asBoolean());
            if (covered && !t.has("scenario") && witness != null) {
                t.set("scenario", Json.tree(witness));
            }
        }

        ObjectNode out = Json.object();
        out.put("schema", SCHEMA);
        out.put("engineVersion", ExperimentRunner.ENGINE_VERSION);
        out.put("name", campaign.path("name").asText("coverage"));
        out.put("controller", base.controllerId());
        out.put("instrumentation", base.instrumentation());
        ObjectNode problem = problemNode.deepCopy();
        baseInstance.normalise(problem);
        out.set("problem", problem);
        ObjectNode algorithm = out.putObject("algorithm");
        algorithm.put("type", algorithmType);
        algorithm.set("params", algorithmParams);
        out.put("budget", total);
        out.put("seed", seed);
        out.put("threads", threads);
        out.put("startedAt", started.toString());
        out.put("finishedAt", Instant.now().toString());
        out.put("cancelled", control.isCancelled());
        ObjectNode summary = out.putObject("summary");
        summary.put("outcomes", coverage.totalOutcomes());
        summary.put("covered", coverage.coveredOutcomes());
        summary.put("coverage", (double) coverage.coveredOutcomes() / coverage.totalOutcomes());
        summary.put("evaluations", total - remaining);
        out.set("targets", targets);
        out.set("scenarioCounts", Json.tree(coverage.report()));
        if (campaign.path("randomBaseline").asBoolean(true) && !control.isCancelled()) {
            out.set("random", randomBaseline(problemNode, baseDir, total, seed, threads, control));
        }
        if (outputDir != null) {
            Path dir = write(outputDir, out);
            out.put("outputDir", dir.toAbsolutePath().toString());
        }
        return out;
    }

    /** Number of targets from position {@code from} onwards that are not covered yet. */
    private static int uncoveredFrom(CoverageAccumulator coverage, List<Branch> branches, int from) {
        int n = 0;
        int index = 0;
        for (Branch b : branches) {
            for (boolean outcome : new boolean[] {true, false}) {
                if (index++ >= from && !coverage.covered(b.id(), outcome)) {
                    n++;
                }
            }
        }
        return Math.max(1, n);
    }

    @SuppressWarnings("unchecked")
    private static long search(ObjectNode t, JsonNode problemNode, Path baseDir, Branch branch, boolean outcome,
                               long budget, long seed, int threads, AlgorithmRegistry.Descriptor descriptor,
                               JsonNode algorithmParams, RunControl control, CoverageAccumulator coverage) {
        ObjectNode config = problemNode.deepCopy();
        ObjectNode params = config.path("params").isObject() ? (ObjectNode) config.path("params") : config.putObject("params");
        params.put("fitness", "branch:" + branch.id() + ":" + outcome);
        params.put("target", "branch");
        ProblemInstance<RealVector> instance = (ProblemInstance<RealVector>) Problems.create(config, baseDir);
        ControllerTestingProblem problem = (ControllerTestingProblem) instance.problem();
        SearchAlgorithm algorithm = descriptor.create(Params.of(Json.toMap(algorithmParams)));
        SearchContext<RealVector> context = SearchContext.builder(instance.problem(), Budget.evaluations(budget), seed)
                .control(control)
                .stopOnTarget(true)
                .maxTargets(1)
                .archiveSize(1)
                .threads(threads)
                .build();
        SearchResult<RealVector> result = SearchAlgorithm.execute(algorithm, context);
        coverage.merge(problem.coverage());
        t.put("budget", budget);
        t.put("evaluations", result.evaluations());
        if (result.targetCount() > 0) {
            Candidate<RealVector> first = result.targets().get(0);
            t.put("status", "targeted");
            t.put("firstEvaluation", result.firstTargetEvaluation());
            t.set("scenario", Json.tree(first.solution().toArray()));
        } else {
            t.put("status", "uncovered");
            if (result.best() != null) {
                Json.putFinite(t, "bestFitness", result.best().fitness());
                t.put("approachLevel", (int) result.best().evaluation().metric("approachLevel", -1));
                Json.putFinite(t, "branchDistance", result.best().evaluation().metric("branchDistance", Double.NaN));
            }
        }
        return result.evaluations();
    }

    @SuppressWarnings("unchecked")
    private static ObjectNode randomBaseline(JsonNode problemNode, Path baseDir, long budget, long seed, int threads,
                                             RunControl control) {
        ProblemInstance<RealVector> instance = (ProblemInstance<RealVector>) Problems.create(problemNode, baseDir);
        ControllerTestingProblem problem = (ControllerTestingProblem) instance.problem();
        SearchContext<RealVector> context = SearchContext.builder(instance.problem(), Budget.evaluations(budget), seed)
                .control(control)
                .archiveSize(1)
                .maxTargets(1)
                .threads(threads)
                .build();
        SearchResult<RealVector> result = SearchAlgorithm.execute(AlgorithmRegistry.create("random-search", Params.of()), context);
        CoverageAccumulator c = problem.coverage();
        ObjectNode out = Json.object();
        out.put("algorithm", "random-search");
        out.put("evaluations", result.evaluations());
        out.put("covered", c.coveredOutcomes());
        out.put("coverage", (double) c.coveredOutcomes() / c.totalOutcomes());
        ArrayNode missed = out.putArray("uncovered");
        for (Branch b : c.branches()) {
            for (boolean outcome : new boolean[] {true, false}) {
                if (!c.covered(b.id(), outcome)) {
                    missed.add(Branch.outcome(b.id(), outcome));
                }
            }
        }
        return out;
    }

    private static Path write(Path root, ObjectNode out) {
        String name = ExperimentRunner.sanitize(out.path("name").asText("coverage"));
        String stamp = out.path("startedAt").asText().replace(":", "").replace("-", "").replaceAll("\\..*", "");
        Path dir = root.resolve(stamp + "-coverage-" + name);
        int suffix = 1;
        while (Files.exists(dir)) {
            dir = root.resolve(stamp + "-coverage-" + name + "-" + suffix++);
        }
        Json.write(dir.resolve("coverage.json"), out);
        List<String> lines = new ArrayList<>();
        lines.add("target,status,budget,evaluations,first_evaluation,approach_level,branch_distance");
        for (JsonNode t : out.path("targets")) {
            lines.add(String.join(",", t.path("label").asText(), t.path("status").asText(),
                    t.path("budget").asText(""), t.path("evaluations").asText(""), t.path("firstEvaluation").asText(""),
                    t.path("approachLevel").asText(""), t.path("branchDistance").asText("")));
        }
        ExperimentRunner.Csv.write(dir.resolve("coverage.csv"), lines);
        return dir;
    }
}
