package org.robosbse.testing.coverage;

import java.util.Arrays;
import java.util.List;

/**
 * Coverage record of one episode: for every decision, how often it was evaluated and the smallest
 * branch distance observed towards each outcome (0 once the outcome was taken).
 *
 * <p>Branch distances follow the usual definitions for relational predicates (Korel; Tracey et al.):
 * for {@code a < b} the distance to {@code true} is {@code a - b + K} when false, and the distance to
 * {@code false} is {@code b - a} when true. Boolean flags without operands have distance 1. A trace
 * belongs to one episode and one thread.
 */
public final class CoverageTrace {

    /** Small constant that makes strict comparisons need a strictly positive step. */
    public static final double K = 1e-9;

    private final List<Branch> branches;
    private final double[] toTrue;
    private final double[] toFalse;
    private final long[] evaluations;

    public CoverageTrace(List<Branch> branches) {
        this.branches = List.copyOf(branches);
        this.toTrue = new double[branches.size()];
        this.toFalse = new double[branches.size()];
        this.evaluations = new long[branches.size()];
        Arrays.fill(toTrue, Double.POSITIVE_INFINITY);
        Arrays.fill(toFalse, Double.POSITIVE_INFINITY);
    }

    /** Records one evaluation of decision {@code slot} and returns {@code outcome}. */
    public boolean record(int slot, boolean outcome, double distanceToTrue, double distanceToFalse) {
        evaluations[slot]++;
        toTrue[slot] = Math.min(toTrue[slot], outcome ? 0 : sanitize(distanceToTrue));
        toFalse[slot] = Math.min(toFalse[slot], outcome ? sanitize(distanceToFalse) : 0);
        return outcome;
    }

    private static double sanitize(double d) {
        return Double.isNaN(d) ? 1.0 : Math.max(K, d);
    }

    public List<Branch> branches() {
        return branches;
    }

    public int slot(String id) {
        for (int i = 0; i < branches.size(); i++) {
            if (branches.get(i).id().equals(id)) {
                return i;
            }
        }
        throw new IllegalArgumentException("unknown decision '" + id + "'");
    }

    public boolean reached(int slot) {
        return evaluations[slot] > 0;
    }

    public long evaluations(int slot) {
        return evaluations[slot];
    }

    public boolean covered(int slot, boolean outcome) {
        return (outcome ? toTrue[slot] : toFalse[slot]) == 0;
    }

    /** Smallest distance to the outcome over the episode; infinite if the decision was never reached. */
    public double distance(int slot, boolean outcome) {
        return outcome ? toTrue[slot] : toFalse[slot];
    }

    /** Number of covered outcomes (at most twice the number of decisions). */
    public int coveredOutcomes() {
        int n = 0;
        for (int i = 0; i < branches.size(); i++) {
            n += (toTrue[i] == 0 ? 1 : 0) + (toFalse[i] == 0 ? 1 : 0);
        }
        return n;
    }
}
