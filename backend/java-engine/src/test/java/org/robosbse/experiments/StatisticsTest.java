package org.robosbse.experiments;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Reference values were computed with SciPy 1.x (mannwhitneyu, asymptotic, continuity correction) and NumPy. */
class StatisticsTest {

    @Test
    void mannWhitneyMatchesScipyWithoutTies() {
        double[] a = {1.1, 2.3, 3.0, 4.2, 5.5, 6.1, 7.7};
        double[] b = {3.3, 4.4, 5.0, 6.6, 7.0, 8.8, 9.9, 10.1};
        Statistics.MannWhitney mw = Statistics.mannWhitney(a, b);
        assertEquals(12.0, mw.u(), 1e-12);
        assertEquals(0.07284885832099205, mw.pValue(), 1e-5);
    }

    @Test
    void mannWhitneyMatchesScipyWithTies() {
        double[] a = {1, 2, 2, 3, 3, 3};
        double[] b = {2, 3, 4, 4, 5};
        Statistics.MannWhitney mw = Statistics.mannWhitney(a, b);
        assertEquals(5.5, mw.u(), 1e-12);
        assertEquals(0.08871369199677616, mw.pValue(), 1e-5);
    }

    @Test
    void identicalSamplesAreNotSignificant() {
        Statistics.MannWhitney mw = Statistics.mannWhitney(new double[] {1, 1, 1}, new double[] {1, 1});
        assertEquals(1.0, mw.pValue());
    }

    @Test
    void a12CountsWinsUnderMinimisation() {
        assertEquals(1.0, Statistics.a12(new double[] {1, 2}, new double[] {3, 4}));
        assertEquals(0.0, Statistics.a12(new double[] {3, 4}, new double[] {1, 2}));
        assertEquals(0.5, Statistics.a12(new double[] {1, 3}, new double[] {1, 3}));
        assertEquals("large", Statistics.a12Magnitude(0.9));
        assertEquals("negligible", Statistics.a12Magnitude(0.52));
    }

    @Test
    void quantilesMatchNumpy() {
        Statistics.Summary s = Statistics.summarize(new double[] {5, 1, 9, 3, 7, 2});
        assertEquals(2.25, s.q1(), 1e-12);
        assertEquals(4.0, s.median(), 1e-12);
        assertEquals(6.5, s.q3(), 1e-12);
        assertEquals(1.0, s.min());
        assertEquals(9.0, s.max());
    }

    @Test
    void holmAdjustmentIsMonotoneAndCapped() {
        assertArrayEquals(new double[] {0.03, 0.06, 0.06}, Statistics.holm(new double[] {0.01, 0.04, 0.03}), 1e-12);
        assertArrayEquals(new double[] {1.0, 1.0}, Statistics.holm(new double[] {0.6, 0.9}), 1e-12);
    }

    @Test
    void normalCdfIsAccurate() {
        assertEquals(0.5, Statistics.normalCdf(0), 1e-7);
        assertEquals(0.9750021048517795, Statistics.normalCdf(1.96), 1e-6);
        assertEquals(0.0013498980316301, Statistics.normalCdf(-3), 1e-7);
    }
}
