package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

class RealVectorSpaceTest {

    private final RealVectorSpace space = new RealVectorSpace(new double[] {0, 0, -1, -1, 5, 5},
            new double[] {10, 10, 1, 1, 6, 6}, null, 2);

    private void assertInside(RealVector v) {
        for (int i = 0; i < v.size(); i++) {
            assertTrue(v.get(i) >= space.lower(i) && v.get(i) <= space.upper(i), "gene " + i + " out of bounds");
        }
    }

    @Test
    void randomSolutionsRespectBounds() {
        Rng rng = new Rng(1);
        for (int i = 0; i < 1000; i++) {
            assertInside(space.random(rng));
        }
    }

    @Test
    void mutationStaysInBoundsAndAlwaysChangesSomething() {
        Rng rng = new Rng(2);
        for (String name : RealVectorSpace.MUTATIONS) {
            MutationOperator<RealVector> m = space.mutation(Params.of("mutation", name, "mutationSigma", 0.5));
            for (int i = 0; i < 500; i++) {
                RealVector parent = space.random(rng);
                RealVector child = m.mutate(parent, rng);
                assertInside(child);
                assertNotEquals(parent, child, name + " returned an unchanged copy");
            }
        }
    }

    @Test
    void blockCrossoverNeverSplitsABlock() {
        Rng rng = new Rng(3);
        RealVector a = new RealVector(new double[] {1, 1, 0.1, 0.1, 5.1, 5.1});
        RealVector b = new RealVector(new double[] {9, 9, 0.9, 0.9, 5.9, 5.9});
        for (String name : List.of("one-point", "two-point", "uniform")) {
            CrossoverOperator<RealVector> x = space.crossover(Params.of("crossover", name));
            for (int i = 0; i < 200; i++) {
                for (RealVector child : x.cross(a, b, rng)) {
                    for (int blk = 0; blk < 3; blk++) {
                        double g0 = child.get(2 * blk);
                        double g1 = child.get(2 * blk + 1);
                        boolean fromA = g0 == a.get(2 * blk) && g1 == a.get(2 * blk + 1);
                        boolean fromB = g0 == b.get(2 * blk) && g1 == b.get(2 * blk + 1);
                        assertTrue(fromA || fromB, name + " split block " + blk);
                    }
                }
            }
        }
    }

    @Test
    void arithmeticCrossoverStaysBetweenParents() {
        Rng rng = new Rng(4);
        RealVector a = space.random(rng);
        RealVector b = space.random(rng);
        for (RealVector c : space.crossover(Params.of("crossover", "arithmetic")).cross(a, b, rng)) {
            for (int i = 0; i < c.size(); i++) {
                assertTrue(c.get(i) >= Math.min(a.get(i), b.get(i)) - 1e-12);
                assertTrue(c.get(i) <= Math.max(a.get(i), b.get(i)) + 1e-12);
            }
        }
    }

    @Test
    void neighbourhoodMovesOneGeneByOneStep() {
        RealVector s = new RealVector(new double[] {5, 5, 0, 0, 5.5, 5.5});
        Neighbourhood<RealVector> n = space.neighbourhood(Params.of("stepSize", 0.1));
        List<RealVector> all = n.all(s);
        assertEquals(12, all.size());
        for (RealVector v : all) {
            int changed = 0;
            for (int i = 0; i < v.size(); i++) {
                if (v.get(i) != s.get(i)) {
                    changed++;
                    assertEquals(0.1 * space.range(i), Math.abs(v.get(i) - s.get(i)), 1e-12);
                }
            }
            assertEquals(1, changed);
        }
        RealVector corner = new RealVector(new double[] {0, 0, -1, -1, 5, 5});
        assertEquals(6, n.all(corner).size(), "moves leaving the box are dropped");
    }

    @Test
    void distanceIsANormalisedMetric() {
        RealVector lo = new RealVector(new double[] {0, 0, -1, -1, 5, 5});
        RealVector hi = new RealVector(new double[] {10, 10, 1, 1, 6, 6});
        assertEquals(0, space.distance(lo, lo));
        assertEquals(1.0, space.distance(lo, hi), 1e-12);
        assertEquals(space.distance(lo, hi), space.distance(hi, lo));
    }
}
