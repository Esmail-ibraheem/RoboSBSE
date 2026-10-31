package org.robosbse.core;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Exact hypervolume indicator for minimisation: the volume of objective space dominated by a
 * point set and bounded by a reference point. Points that do not strictly dominate the reference
 * point contribute nothing (so the choice of reference point must be documented with every result).
 *
 * <p>Two objectives use an O(n log n) sweep; more objectives use the "hypervolume by slicing
 * objectives" recursion, O(n^(m-1) log n) — exact and fast enough for archives of a few hundred
 * points in three objectives.
 */
public final class Hypervolume {

    private Hypervolume() {
    }

    /**
     * Hypervolume as a fraction of the box between the origin and the reference point (all
     * objectives here are non-negative), so values lie in [0, 1] and are comparable across problems
     * with different units. Falls back to the raw volume if a reference coordinate is not positive.
     */
    public static double normalised(List<double[]> points, double[] reference) {
        double box = 1;
        for (double r : reference) {
            if (!(r > 0)) {
                return of(points, reference);
            }
            box *= r;
        }
        return of(points, reference) / box;
    }

    public static double of(List<double[]> points, double[] reference) {
        List<double[]> inside = new ArrayList<>();
        for (double[] p : points) {
            boolean dominatesReference = true;
            for (int i = 0; i < reference.length; i++) {
                if (!(p[i] < reference[i])) {
                    dominatesReference = false;
                    break;
                }
            }
            if (dominatesReference) {
                inside.add(p);
            }
        }
        return slice(inside, reference, reference.length);
    }

    private static double slice(List<double[]> points, double[] ref, int m) {
        if (points.isEmpty()) {
            return 0;
        }
        if (m == 1) {
            double min = Double.POSITIVE_INFINITY;
            for (double[] p : points) {
                min = Math.min(min, p[0]);
            }
            return ref[0] - min;
        }
        if (m == 2) {
            List<double[]> sorted = new ArrayList<>(points);
            sorted.sort(Comparator.comparingDouble(p -> p[0]));
            double volume = 0;
            double bestY = ref[1];
            for (int i = 0; i < sorted.size(); i++) {
                double y = sorted.get(i)[1];
                if (y < bestY) {
                    bestY = y;
                }
                double nextX = i + 1 < sorted.size() ? sorted.get(i + 1)[0] : ref[0];
                volume += (nextX - sorted.get(i)[0]) * (ref[1] - bestY);
            }
            return volume;
        }
        int last = m - 1;
        List<double[]> sorted = new ArrayList<>(points);
        sorted.sort(Comparator.comparingDouble(p -> p[last]));
        double volume = 0;
        List<double[]> active = new ArrayList<>();
        for (int i = 0; i < sorted.size(); i++) {
            active.add(sorted.get(i));
            double low = sorted.get(i)[last];
            double high = i + 1 < sorted.size() ? sorted.get(i + 1)[last] : ref[last];
            if (high > low) {
                volume += (high - low) * slice(active, ref, m - 1);
            }
        }
        return volume;
    }
}
