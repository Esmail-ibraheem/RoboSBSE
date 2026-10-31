package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;

class ParetoTest {

    private static Evaluation eval(double cv, double... f) {
        return new Evaluation(f.length > 0 ? f[0] : 0, cv, Map.of(), f);
    }

    @Test
    void dominanceIsStrictAndPartial() {
        assertTrue(Pareto.dominates(new double[] {1, 2}, new double[] {2, 2}));
        assertFalse(Pareto.dominates(new double[] {1, 2}, new double[] {1, 2}), "equal vectors do not dominate");
        assertFalse(Pareto.dominates(new double[] {1, 3}, new double[] {2, 2}), "incomparable");
        assertFalse(Pareto.dominates(new double[] {2, 2}, new double[] {1, 3}));
    }

    @Test
    void constrainedDominationPrefersFeasibilityThenLowerViolation() {
        assertTrue(Pareto.constrainedDominates(eval(0, 100, 100), eval(0.1, 1, 1)));
        assertTrue(Pareto.constrainedDominates(eval(1, 9, 9), eval(2, 1, 1)));
        assertFalse(Pareto.constrainedDominates(eval(2, 1, 1), eval(1, 9, 9)));
        assertTrue(Pareto.constrainedDominates(eval(0, 1, 1), eval(0, 1, 2)));
    }

    @Test
    void nonDominatedSortingFindsTheFronts() {
        List<Evaluation> e = List.of(eval(0, 1, 5), eval(0, 2, 2), eval(0, 5, 1), eval(0, 3, 3), eval(0, 6, 6), eval(1, 0, 0));
        List<List<Integer>> fronts = Pareto.sort(e);
        assertEquals(List.of(0, 1, 2), fronts.get(0).stream().sorted().toList());
        assertEquals(List.of(3), fronts.get(1));
        assertEquals(List.of(4), fronts.get(2));
        assertEquals(List.of(5), fronts.get(3), "the infeasible point comes last despite better objectives");
        for (List<Integer> front : fronts) {
            for (int a : front) {
                for (int b : front) {
                    assertFalse(Pareto.constrainedDominates(e.get(a), e.get(b)), "a front must be mutually non-dominated");
                }
            }
        }
    }

    @Test
    void crowdingDistanceGivesBoundariesInfinity() {
        double[] d = Pareto.crowdingDistance(List.of(new double[] {0, 4}, new double[] {1, 2}, new double[] {4, 0}));
        assertEquals(Double.POSITIVE_INFINITY, d[0]);
        assertEquals(Double.POSITIVE_INFINITY, d[2]);
        assertEquals(4.0 / 4 + 4.0 / 4, d[1], 1e-12);
    }

    @Test
    void hypervolumeKnownValues() {
        List<double[]> staircase = List.of(new double[] {1, 3}, new double[] {2, 2}, new double[] {3, 1});
        assertEquals(6.0, Hypervolume.of(staircase, new double[] {4, 4}), 1e-12);
        assertEquals(6.0 / 16, Hypervolume.normalised(staircase, new double[] {4, 4}), 1e-12);
        assertEquals(1.0, Hypervolume.of(List.<double[]>of(new double[] {1, 1, 1}), new double[] {2, 2, 2}), 1e-12);
        List<double[]> two = List.of(new double[] {0, 0, 1}, new double[] {1, 1, 0});
        assertEquals(5.0, Hypervolume.of(two, new double[] {2, 2, 2}), 1e-12);
        assertEquals(0.0, Hypervolume.of(List.<double[]>of(new double[] {5, 0}), new double[] {4, 4}),
                "points beyond the reference contribute nothing");
    }

    @Test
    void threeDimensionalHypervolumeMatchesMonteCarlo() {
        Rng rng = new Rng(4);
        List<double[]> pts = new ArrayList<>();
        for (int i = 0; i < 25; i++) {
            pts.add(new double[] {rng.nextDouble(), rng.nextDouble(), rng.nextDouble()});
        }
        double exact = Hypervolume.of(pts, new double[] {1, 1, 1});
        int inside = 0;
        int samples = 400_000;
        for (int s = 0; s < samples; s++) {
            double[] q = {rng.nextDouble(), rng.nextDouble(), rng.nextDouble()};
            for (double[] p : pts) {
                if (p[0] <= q[0] && p[1] <= q[1] && p[2] <= q[2]) {
                    inside++;
                    break;
                }
            }
        }
        assertEquals((double) inside / samples, exact, 0.005);
    }

    @Test
    void archiveKeepsOnlyFeasibleNonDominatedPointsAndItsExtremes() {
        ParetoArchive<String> archive = new ParetoArchive<>(3);
        assertTrue(archive.offer(new Candidate<>("a", eval(0, 1, 5), 1)));
        assertTrue(archive.offer(new Candidate<>("b", eval(0, 5, 1), 2)));
        assertFalse(archive.offer(new Candidate<>("c", eval(0, 6, 6), 3)), "dominated");
        assertFalse(archive.offer(new Candidate<>("d", eval(0.5, 0, 0), 4)), "infeasible");
        assertTrue(archive.offer(new Candidate<>("e", eval(0, 2, 2), 5)));
        archive.offer(new Candidate<>("f", eval(0, 3, 1.5), 6)); // overflow: the most crowded interior point goes
        assertEquals(3, archive.size());
        List<String> kept = archive.members().stream().map(Candidate::solution).toList();
        assertTrue(kept.contains("a") && kept.contains("b"), "extremes survive truncation: " + kept);
        assertTrue(archive.offer(new Candidate<>("g", eval(0, 0.5, 0.5), 7)));
        assertEquals(List.of("g"), archive.members().stream().map(Candidate::solution).toList(), "g dominates everything");
    }
}
