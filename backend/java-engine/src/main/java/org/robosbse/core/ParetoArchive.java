package org.robosbse.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Bounded archive of mutually non-dominated <em>feasible</em> candidates — the Pareto-front
 * approximation of a run. Every algorithm gets one (the {@link SearchContext} feeds it every
 * evaluation), so single-objective and multi-objective algorithms can be compared on the same
 * indicator. When the archive overflows, the member with the smallest crowding distance (the most
 * crowded interior point) is removed; boundary points are never removed.
 */
public final class ParetoArchive<S> {

    private final int capacity;
    private final List<Candidate<S>> members = new ArrayList<>();

    public ParetoArchive(int capacity) {
        this.capacity = Math.max(2, capacity);
    }

    /** Offers a candidate; returns true if it entered the archive. */
    public boolean offer(Candidate<S> candidate) {
        Evaluation e = candidate.evaluation();
        double[] f = e.objectives();
        if (!e.feasible() || f.length == 0) {
            return false;
        }
        for (Candidate<S> m : members) {
            double[] g = m.evaluation().objectives();
            if (Pareto.dominates(g, f) || Arrays.equals(g, f)) {
                return false;
            }
        }
        members.removeIf(m -> Pareto.dominates(f, m.evaluation().objectives()));
        members.add(candidate);
        if (members.size() > capacity) {
            List<double[]> points = points();
            double[] crowding = Pareto.crowdingDistance(points);
            int worst = 0;
            for (int i = 1; i < crowding.length; i++) {
                if (crowding[i] < crowding[worst]) {
                    worst = i;
                }
            }
            members.remove(worst);
        }
        return members.contains(candidate);
    }

    public List<Candidate<S>> members() {
        return List.copyOf(members);
    }

    public List<double[]> points() {
        List<double[]> points = new ArrayList<>(members.size());
        for (Candidate<S> m : members) {
            points.add(m.evaluation().objectives());
        }
        return points;
    }

    public int size() {
        return members.size();
    }

    /** Normalised hypervolume (fraction of the reference box), or NaN without a reference point. */
    public double hypervolume(double[] reference) {
        return reference == null ? Double.NaN : Hypervolume.normalised(points(), reference);
    }
}
