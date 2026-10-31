package org.robosbse.experiments;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Candidate;
import org.robosbse.core.SearchResult;
import org.robosbse.improvement.ImprovementProblem;
import org.robosbse.patching.Patch;
import org.robosbse.patching.SourceCatalog;
import org.robosbse.patching.Subject;
import org.robosbse.patching.Suites;
import org.robosbse.repair.RepairProblem;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.WorldMap;

/** Problem instances for program repair and genetic improvement (source patches). */
final class PatchInstances {

    private PatchInstances() {
    }

    static Map<String, Object> subjectInfo(Subject s, SourceCatalog catalog) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("id", s.id());
        m.put("name", s.name());
        m.put("className", s.className());
        if (s.defectKind() != null) {
            m.put("defectKind", s.defectKind());
            m.put("defect", s.defect());
        }
        m.put("scope", s.scope().isEmpty() ? "whole class" : s.scope());
        m.put("mutationPoints", catalog.points().size());
        m.put("singleEdits", catalog.singleEdits());
        return m;
    }

    static List<Map<String, Object>> testTable(List<Suites.TestCase> tests, JsonNode result, int failing) {
        List<Map<String, Object>> out = new ArrayList<>();
        for (int i = 0; i < tests.size(); i++) {
            JsonNode r = result.path("tests").path(i);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("name", tests.get(i).name());
            m.put("kind", i < failing ? "failing" : "passing");
            m.put("requireGoal", tests.get(i).requireGoal());
            m.put("passed", r.path("passed").asBoolean(false));
            m.put("outcome", r.path("outcome").asText(Suites.status(result)));
            m.put("violated", r.path("violated").isArray() ? r.path("violated") : List.of());
            out.add(m);
        }
        return out;
    }

    /** Program repair: patches ranked by held-out validation, minimised, with side-by-side diffs. */
    static final class Repair implements ProblemInstance<Patch> {
        private final RepairProblem problem;

        Repair(RepairProblem problem) {
            this.problem = problem;
        }

        @Override
        public RepairProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return MapLibrary.builtin("testing-arena");
        }

        @Override
        public void normalise(ObjectNode problemNode) {
            // The subject is referenced by id or file; the tests are regenerated deterministically from the seeds.
        }

        @Override
        public Map<String, Object> replay(Patch patch) {
            JsonNode r = problem.run(patch);
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("patch", problem.view(patch));
            out.put("status", Suites.status(r));
            out.put("error", r.path("error").asText(""));
            out.put("tests", testTable(problem.searchTests(), r, problem.failingTests()));
            out.put("diff", SourceCatalog.diff(problem.subject().source(), problem.catalog().apply(patch)));
            return out;
        }

        @Override
        public Map<String, Object> reference() {
            Map<String, Object> original = new LinkedHashMap<>();
            original.put("tests", testTable(problem.searchTests(), problem.originalResults(), problem.failingTests()));
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("original", original);
            out.put("note", "The unpatched subject on the search tests; the reference fix is used only to judge patches.");
            return out;
        }

        @Override
        public String targetsField() {
            return "patches";
        }

        /**
         * Distinct plausible patches in discovery order, each minimised (ddmin over its edits; these
         * re-evaluations are cached or extra and not charged to the search budget), validated on the
         * held-out tests and ranked: passes held-out tests, then equivalent to the reference fix, then
         * fewer edits.
         */
        @Override
        public List<Object> describeTargets(SearchResult<Patch> result, Map<String, Object> provenance, boolean minimize,
                                            int limit) {
            Set<Patch> seen = new LinkedHashSet<>();
            List<Map<String, Object>> patches = new ArrayList<>();
            for (Candidate<Patch> c : result.targets()) {
                if (patches.size() >= limit) {
                    break;
                }
                Patch found = c.solution();
                Patch reduced = minimize ? problem.minimise(found) : found;
                if (!seen.add(reduced)) {
                    continue;
                }
                patches.add(describe(reduced, found, c.index()));
            }
            patches.sort(Comparator.<Map<String, Object>, Boolean>comparing(m -> !(Boolean) m.get("passesHeldOut"))
                    .thenComparing(m -> !(Boolean) m.get("equivalentToReferenceFix"))
                    .thenComparingInt(m -> (Integer) m.get("size")));
            List<Object> out = new ArrayList<>();
            for (int i = 0; i < patches.size(); i++) {
                patches.get(i).put("rank", i + 1);
                out.add(patches.get(i));
            }
            return out;
        }

        Map<String, Object> describe(Patch patch, Patch found, long evaluation) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("patch", patch.toString());
            m.put("edits", problem.catalog().describe(patch));
            m.put("size", patch.size());
            m.put("found", found.toString());
            m.put("foundAtEvaluation", evaluation);
            m.putAll(problem.validate(patch));
            m.put("changes", SourceCatalog.changes(problem.subject().source(), problem.catalog().apply(patch)));
            m.put("diff", SourceCatalog.diff(problem.subject().source(), problem.catalog().apply(patch)));
            return m;
        }

        @Override
        public Patch parse(JsonNode genome) {
            return Patch.parse(genome.isTextual() ? genome.asText() : genome.path("patch").asText());
        }

        /** For the best patch: whether it is plausible, and (if so) its minimised form's held-out validation. */
        @Override
        public Map<String, Object> bestExtras(Patch best) {
            Map<String, Object> out = new LinkedHashMap<>();
            boolean plausible = problem.plausible(best);
            out.put("plausible", plausible);
            if (plausible) {
                Patch reduced = problem.minimise(best);
                out.put("minimisedPatch", reduced.toString());
                out.put("minimisedSize", reduced.size());
                out.put("edits", problem.catalog().describe(reduced));
                out.putAll(problem.validate(reduced));
            }
            return out;
        }

        @Override
        public Map<String, Object> runExtras() {
            Map<String, Object> repair = new LinkedHashMap<>();
            repair.put("subject", subjectInfo(problem.subject(), problem.catalog()));
            repair.put("searchTests", Map.of("failing", problem.failingTests(),
                    "passing", problem.searchTests().size() - problem.failingTests()));
            repair.put("heldOutTests", Map.of("failing", problem.heldOutFailing(),
                    "passing", problem.heldOutTests().size() - problem.heldOutFailing()));
            repair.put("candidates", problem.pool().stats());
            repair.put("originalSource", problem.subject().source());
            repair.put("note", "Passing the tests does not prove a patch correct: plausible patches are validated on "
                    + "held-out tests and compared with the reference fix.");
            return Map.of("repair", repair);
        }
    }

    /** Genetic improvement: baseline vs improved behaviour, correctness on held-out tests, diffs. */
    static final class Improvement implements ProblemInstance<Patch> {
        private final ImprovementProblem problem;

        Improvement(ImprovementProblem problem) {
            this.problem = problem;
        }

        @Override
        public ImprovementProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return MapLibrary.builtin("testing-arena");
        }

        @Override
        public void normalise(ObjectNode problemNode) {
            // Subject by reference, tests regenerated from the seeds.
        }

        @Override
        public Map<String, Object> replay(Patch patch) {
            JsonNode r = problem.run(patch);
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("patch", problem.view(patch));
            out.put("status", Suites.status(r));
            out.put("metrics", problem.measure(r));
            out.put("baseline", problem.baselineMetrics());
            out.put("diff", SourceCatalog.diff(problem.subject().source(), problem.catalog().apply(patch)));
            return out;
        }

        @Override
        public Map<String, Object> reference() {
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("baseline", problem.baselineMetrics());
            out.put("fitness", problem.baselineFitness());
            out.put("note", "The unchanged controller; time and work ratios are relative to it.");
            return out;
        }

        @Override
        public String targetsField() {
            return "improvements";
        }

        /**
         * The best distinct improvements (up to {@code limit}, by fitness), each validated on the held-out
         * correctness tests and ranked: passing all held-out tests first, then fitness. An improvement that
         * passes the search suite but fails a held-out test exposes a gap in the suite, not a valid change.
         */
        @Override
        public List<Object> describeTargets(SearchResult<Patch> result, Map<String, Object> provenance, boolean minimize,
                                            int limit) {
            List<Candidate<Patch>> targets = new ArrayList<>(result.targets());
            targets.sort(Comparator.comparingDouble(Candidate::fitness));
            Set<Patch> seen = new LinkedHashSet<>();
            List<Map<String, Object>> found = new ArrayList<>();
            for (Candidate<Patch> c : targets) {
                if (found.size() >= limit || !seen.add(c.solution())) {
                    continue;
                }
                Map<String, Object> m = new LinkedHashMap<>();
                m.put("patch", c.solution().toString());
                m.put("edits", problem.catalog().describe(c.solution()));
                m.put("fitness", c.fitness());
                m.put("metrics", c.evaluation().metrics());
                m.put("foundAtEvaluation", c.index());
                m.putAll(problem.validate(c.solution()));
                m.put("changes", SourceCatalog.changes(problem.subject().source(), problem.catalog().apply(c.solution())));
                m.put("diff", SourceCatalog.diff(problem.subject().source(), problem.catalog().apply(c.solution())));
                found.add(m);
            }
            found.sort(Comparator.<Map<String, Object>, Boolean>comparing(m -> !(Boolean) m.get("passesHeldOut"))
                    .thenComparingDouble(m -> (Double) m.get("fitness")));
            List<Object> out = new ArrayList<>();
            for (int i = 0; i < found.size(); i++) {
                found.get(i).put("rank", i + 1);
                out.add(found.get(i));
            }
            return out;
        }

        @Override
        public Patch parse(JsonNode genome) {
            return Patch.parse(genome.isTextual() ? genome.asText() : genome.path("patch").asText());
        }

        @Override
        public Map<String, Object> bestExtras(Patch best) {
            Map<String, Object> out = new LinkedHashMap<>();
            Map<String, Double> m = problem.measure(problem.run(best));
            out.put("baseline", problem.baselineMetrics());
            out.put("improved", m);
            out.put("improvement", m.get("valid") > 0 && problem.isTarget(problem.evaluate(best)));
            out.put("edits", problem.catalog().describe(best));
            if (m.get("valid") > 0) {
                out.putAll(problem.validate(best));
            }
            out.put("changes", SourceCatalog.changes(problem.subject().source(), problem.catalog().apply(best)));
            return out;
        }

        @Override
        public boolean success(SearchResult<Patch> result) {
            return result.targetCount() > 0;
        }

        @Override
        public Map<String, Object> runExtras() {
            Map<String, Object> gi = new LinkedHashMap<>();
            gi.put("subject", subjectInfo(problem.subject(), problem.catalog()));
            gi.put("tests", problem.tests().size());
            gi.put("heldOutTests", problem.heldOutTests().size());
            gi.put("candidates", problem.pool().stats());
            gi.put("originalSource", problem.subject().source());
            return Map.of("improvement", gi);
        }
    }
}
