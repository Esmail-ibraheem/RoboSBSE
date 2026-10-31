package org.robosbse.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Pareto-dominance utilities for minimised objective vectors, with Deb's constraint handling.
 *
 * <p>Constrained domination: a feasible solution dominates every infeasible one; between two
 * infeasible solutions the one with the smaller constraint violation dominates; between two
 * feasible solutions ordinary Pareto dominance applies.
 */
public final class Pareto {

    private Pareto() {
    }

    /** a dominates b: no worse in every objective and strictly better in at least one. */
    public static boolean dominates(double[] a, double[] b) {
        boolean strictly = false;
        for (int i = 0; i < a.length; i++) {
            if (a[i] > b[i]) {
                return false;
            }
            if (a[i] < b[i]) {
                strictly = true;
            }
        }
        return strictly;
    }

    public static boolean constrainedDominates(Evaluation a, Evaluation b) {
        boolean fa = a.feasible();
        boolean fb = b.feasible();
        if (fa && !fb) {
            return true;
        }
        if (!fa && !fb) {
            return a.constraintViolation() < b.constraintViolation();
        }
        if (!fa) {
            return false;
        }
        return dominates(a.objectives(), b.objectives());
    }

    /**
     * Fast non-dominated sorting (Deb et al., 2002) under constrained domination. Returns the
     * fronts as lists of indices into {@code evaluations}; front 0 is non-dominated. O(M N^2).
     */
    public static List<List<Integer>> sort(List<Evaluation> evaluations) {
        int n = evaluations.size();
        List<List<Integer>> dominatedBy = new ArrayList<>(n);
        int[] dominationCount = new int[n];
        List<List<Integer>> fronts = new ArrayList<>();
        List<Integer> first = new ArrayList<>();
        for (int p = 0; p < n; p++) {
            dominatedBy.add(new ArrayList<>());
        }
        for (int p = 0; p < n; p++) {
            for (int q = p + 1; q < n; q++) {
                if (constrainedDominates(evaluations.get(p), evaluations.get(q))) {
                    dominatedBy.get(p).add(q);
                    dominationCount[q]++;
                } else if (constrainedDominates(evaluations.get(q), evaluations.get(p))) {
                    dominatedBy.get(q).add(p);
                    dominationCount[p]++;
                }
            }
        }
        for (int p = 0; p < n; p++) {
            if (dominationCount[p] == 0) {
                first.add(p);
            }
        }
        List<Integer> current = first;
        while (!current.isEmpty()) {
            fronts.add(current);
            List<Integer> next = new ArrayList<>();
            for (int p : current) {
                for (int q : dominatedBy.get(p)) {
                    if (--dominationCount[q] == 0) {
                        next.add(q);
                    }
                }
            }
            current = next;
        }
        return fronts;
    }

    /**
     * Crowding distance of each member of a front (objective vectors): the sum over objectives of
     * the normalised gap between its neighbours. Boundary solutions get +infinity.
     */
    public static double[] crowdingDistance(List<double[]> front) {
        int n = front.size();
        double[] distance = new double[n];
        if (n == 0) {
            return distance;
        }
        int m = front.get(0).length;
        Integer[] order = new Integer[n];
        for (int k = 0; k < m; k++) {
            final int obj = k;
            for (int i = 0; i < n; i++) {
                order[i] = i;
            }
            Arrays.sort(order, Comparator.comparingDouble(i -> front.get(i)[obj]));
            double min = front.get(order[0])[obj];
            double max = front.get(order[n - 1])[obj];
            distance[order[0]] = Double.POSITIVE_INFINITY;
            distance[order[n - 1]] = Double.POSITIVE_INFINITY;
            if (max - min <= 0) {
                continue;
            }
            for (int i = 1; i < n - 1; i++) {
                distance[order[i]] += (front.get(order[i + 1])[obj] - front.get(order[i - 1])[obj]) / (max - min);
            }
        }
        return distance;
    }
}
