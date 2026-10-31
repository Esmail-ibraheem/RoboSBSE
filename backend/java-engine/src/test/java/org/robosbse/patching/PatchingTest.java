package org.robosbse.patching;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.geometry.Vec2;
import org.robosbse.patching.sandbox.WorkerPool;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.testing.Scenario;

class PatchingTest {

    /** A single-edit patch that turns the faulty subject into its reference fix (or an exact equivalent). */
    static Patch fixingEdit(Subject s, SourceCatalog catalog) {
        String fix = SourceCatalog.normalise(s.fixedSource());
        for (MutationPoint p : catalog.points()) {
            for (int c = 0; c < p.alternatives().size(); c++) {
                Patch patch = Patch.of(List.of(new Patch.Edit(p.index(), c)));
                if (SourceCatalog.normalise(catalog.apply(patch)).equals(fix)) {
                    return patch;
                }
            }
        }
        return null;
    }

    @ParameterizedTest
    @ValueSource(strings = {"rb1-stopping-distance", "rb2-comparison", "rb3-turn-direction", "rb5-sensor-mirror"})
    void theReferenceFixIsOneAllowedEditAway(String id) {
        Subject s = Subject.resolve(id, null);
        SourceCatalog catalog = new SourceCatalog(s.source(), s.scope());
        assertTrue(catalog.points().size() > 100, "whole-class scope: " + catalog.points().size());
        Patch fix = fixingEdit(s, catalog);
        assertNotNull(fix, "no single edit reproduces the reference fix of " + id);
        List<SourceCatalog.DiffRow> changes = SourceCatalog.changes(s.source(), catalog.apply(fix));
        assertEquals(1, changes.size(), "lexical preservation: only the edited line changes");
    }

    @Test
    void boundaryCheckFixIsReachableAsAnEquivalentConstant() {
        Subject s = Subject.resolve("rb4-boundary-check", null);
        SourceCatalog catalog = new SourceCatalog(s.source(), s.scope());
        boolean found = false;
        for (MutationPoint p : catalog.points()) {
            if (p.kind() == MutationPoint.Kind.LITERAL && p.original().equals("1.5")) {
                int c = p.alternatives().indexOf("1.0");
                assertTrue(c >= 0);
                // The template has two 1.5 literals (boundary check, corner-guard gap); one edit must be the fix.
                String patched = catalog.apply(Patch.of(List.of(new Patch.Edit(p.index(), c))));
                found |= patched.contains("if (limit > robot.maxSpeed() * 1.0)");
            }
        }
        assertTrue(found);
    }

    @Test
    void patchesRoundTripAndConflictingEditsAreRejected() {
        Subject s = Subject.resolve("rb2-comparison", null);
        SourceCatalog catalog = new SourceCatalog(s.source(), s.scope());
        Patch p = Patch.parse("12:1,3:0");
        assertEquals("3:0,12:1", p.toString());
        assertEquals(p, Patch.parse(p.toString()));
        MutationPoint condition = catalog.points().stream()
                .filter(m -> m.kind() == MutationPoint.Kind.CONDITION && !m.conflicts().isEmpty()).findFirst().orElseThrow();
        int nested = condition.conflicts().get(0);
        Patch conflicting = Patch.of(List.of(new Patch.Edit(condition.index(), 0), new Patch.Edit(nested, 0)));
        assertFalse(catalog.compatible(conflicting));
        assertThrows(IllegalArgumentException.class, () -> catalog.apply(conflicting));
        PatchSpace space = new PatchSpace(catalog, 3);
        Rng rng = new Rng(4);
        for (int i = 0; i < 200; i++) {
            Patch r = space.mutation(Params.empty()).mutate(space.random(rng), rng);
            assertTrue(catalog.compatible(r) && r.size() <= 3 && !r.isEmpty());
        }
    }

    private static List<Suites.TestCase> openDrives() {
        List<Suites.TestCase> tests = new ArrayList<>();
        tests.add(new Suites.TestCase("open", new Scenario(new Pose(2, 6, 0), new Vec2(10, 6), 0, 0, 1.0, 0.8, 1.0, List.of()), true));
        tests.add(new Suites.TestCase("diagonal", new Scenario(new Pose(2, 2, 0.5), new Vec2(9, 9), 0, 0, 1.0, 0.6, 1.0, List.of()), true));
        return tests;
    }

    @Test
    void workersCompileRunAndIsolateCandidates() {
        Subject s = Subject.resolve("rb2-comparison", null);
        try (WorkerPool pool = new WorkerPool(Suites.setup(MapLibrary.builtin("testing-arena"),
                Suites.testingParams("safety", 1200), RobotSpec.DEFAULT, openDrives(), List.of()), 1, 8000)) {
            JsonNode faulty = pool.evaluate(s.className(), s.source(), false);
            assertTrue(Suites.ran(faulty));
            assertFalse(Suites.passed(faulty, 0), "reports completion at the start");
            assertTrue(faulty.path("tests").path(0).path("violated").toString().contains("completion"));
            JsonNode fixed = pool.evaluate(s.className(), s.fixedSource(), false);
            assertTrue(Suites.passed(fixed, 0) && Suites.passed(fixed, 1), fixed.toString());

            JsonNode broken = pool.evaluate(s.className(), s.source().replace("return Command.complete();", "return 42;"), false);
            assertFalse(broken.path("compiled").asBoolean());
            assertTrue(broken.path("error").asText().contains("line"));

            String looping = s.fixedSource().replace("return Command.drive(v, omega);",
                    "while (v >= 0) { v = Math.abs(v); } return Command.drive(v, omega);");
            JsonNode hung = pool.evaluate(s.className(), looping, false);
            assertTrue(hung.path("timeout").asBoolean(), hung.toString());
            JsonNode after = pool.evaluate(s.className(), s.fixedSource() + "\n", false);
            assertTrue(Suites.passed(after, 0), "a fresh worker replaces the killed one");
            assertEquals(1L, pool.stats().get("timeouts"));
            assertEquals(s.fixedSource(), s.fixedSource());
            assertEquals(fixed, pool.evaluate(s.className(), s.fixedSource(), false), "cached");
        }
    }

    @Test
    void repairRoundTripOnTheComparisonBug() {
        try (org.robosbse.repair.RepairProblem p = new org.robosbse.repair.RepairProblem(Subject.resolve("rb2-comparison", null),
                Params.of("poolSize", 40, "passingTests", 5, "heldOutPassing", 10, "failingTests", 3, "heldOutFailing", 3),
                RobotSpec.DEFAULT)) {
            assertEquals(3, p.failingTests());
            Patch fix = fixingEdit(p.subject(), p.catalog());
            assertTrue(p.plausible(fix));
            assertFalse(p.plausible(Patch.empty()));
            Patch withNoise = fix.with(new Patch.Edit(fix.edits().get(0).point() == 0 ? 1 : 0, 0));
            if (p.catalog().compatible(withNoise) && p.plausible(withNoise)) {
                assertEquals(fix, p.minimise(withNoise), "ddmin removes the irrelevant edit");
            }
            var validation = p.validate(fix);
            assertEquals(true, validation.get("passesHeldOut"));
            assertEquals(true, validation.get("equivalentToReferenceFix"));
            assertEquals(true, validation.get("identicalToReferenceFix"));
        }
    }
}
