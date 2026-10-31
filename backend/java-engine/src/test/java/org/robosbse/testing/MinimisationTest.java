package org.robosbse.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.core.Params;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;

class MinimisationTest {

    private static List<Integer> range(int n) {
        List<Integer> out = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            out.add(i);
        }
        return out;
    }

    @Test
    void ddminFindsTheMinimalFailureInducingSubset() {
        int[] tests = {0};
        List<Integer> result = Failures.ddmin(range(16), s -> {
            tests[0]++;
            return s.contains(3) && s.contains(11);
        });
        assertEquals(List.of(3, 11), result);
        assertTrue(tests[0] < 60, "ddmin should need far fewer than 2^16 tests: " + tests[0]);
        assertEquals(List.of(7), Failures.ddmin(range(9), s -> s.contains(7)));
        assertEquals(List.of(), Failures.ddmin(range(5), s -> true));
        assertEquals(List.of(), Failures.ddmin(List.<Integer>of(), s -> true));
    }

    @Test
    void ddminResultIsOneMinimal() {
        // Failure needs any two of {2, 5, 9}.
        List<Integer> result = Failures.ddmin(range(12), s -> (s.contains(2) ? 1 : 0) + (s.contains(5) ? 1 : 0)
                + (s.contains(9) ? 1 : 0) >= 2);
        assertEquals(2, result.size());
        for (Integer removed : result) {
            List<Integer> fewer = new ArrayList<>(result);
            fewer.remove(removed);
            assertTrue((fewer.contains(2) ? 1 : 0) + (fewer.contains(5) ? 1 : 0) + (fewer.contains(9) ? 1 : 0) < 2);
        }
    }

    @Test
    void roundingSnapsGeometryToTheGrid() {
        Scenario s = new Scenario(new Pose(2.04, 6.96, 0.123), new Vec2(10.01, 5.55), 0.01, 0.2, 0.5, 0.9, 1.0,
                List.<Shape>of(new Circle(4.04, 4.06, 0.31), new Rect(1.01, 1.02, 0.49, 0.5)));
        Scenario r = Failures.round(s, 0.1);
        assertEquals(2.0, r.start().x(), 1e-9);
        assertEquals(7.0, r.start().y(), 1e-9);
        assertEquals(0.1, r.start().theta(), 1e-9);
        Circle c = (Circle) r.obstacles().get(0);
        assertEquals(4.0, c.x(), 1e-9);
        assertEquals(4.1, c.y(), 1e-9);
        assertEquals(0.3, c.r(), 1e-9);
        assertEquals(0.5, ((Rect) r.obstacles().get(1)).w(), 1e-9);
        assertEquals(s.sensorNoise(), r.sensorNoise());
    }

    @Test
    void minimisedBug1FailureStillFailsAndIsSimpler() {
        ControllerTestingProblem p = new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                Params.of("controller", "reactive-bug1-dropout", "properties", "safety", "fitness", "collision"),
                Params.empty(), RobotSpec.DEFAULT);
        org.robosbse.core.Rng rng = new org.robosbse.core.Rng(9);
        Scenario failing = null;
        for (int i = 0; i < 3000 && failing == null; i++) {
            Scenario s = p.decode(p.space().random(rng));
            if (p.invalidity(s) == 0 && s.obstacles().size() >= 2 && s.scanDropout() > 0.1
                    && p.execute(s, "reactive-bug1-dropout", false).violated().contains("collision")) {
                failing = s;
            }
        }
        assertTrue(failing != null);
        Failures.Minimized m = Failures.minimize(p, failing, "collision");
        assertTrue(p.execute(m.scenario(), "reactive-bug1-dropout", false).violated().contains("collision"));
        assertTrue(p.obstaclesClearOfEndpoints(m.scenario()));
        assertTrue(m.scenario().obstacles().size() <= failing.obstacles().size());
        assertTrue(m.scenario().scanDropout() <= failing.scanDropout());
        assertTrue(m.scenario().scanDropout() > 0, "BUG-1 needs lost scans");
        assertTrue(m.scenario().battery() >= failing.battery());
        assertTrue(!m.steps().isEmpty());
    }
}
