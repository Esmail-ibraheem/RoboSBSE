package org.robosbse.testing.coverage;

/**
 * Instrumentation probe used inside controller code. Each method evaluates a condition, returns its
 * value unchanged, and — when a {@link CoverageTrace} is attached — records the outcome and the branch
 * distances. Without a trace the methods only compute the comparison, so instrumented controllers
 * behave exactly like uninstrumented ones.
 *
 * <p>Decisions are addressed by slot (their index in the controller's declared branch list) to keep
 * the per-step cost low.
 */
public final class Probe {

    private CoverageTrace trace;

    public void attach(CoverageTrace trace) {
        this.trace = trace;
    }

    public CoverageTrace trace() {
        return trace;
    }

    /** {@code a < b}. */
    public boolean lt(int slot, double a, double b) {
        boolean outcome = a < b;
        if (trace != null) {
            trace.record(slot, outcome, a - b + CoverageTrace.K, b - a);
        }
        return outcome;
    }

    /** {@code a <= b}. */
    public boolean le(int slot, double a, double b) {
        boolean outcome = a <= b;
        if (trace != null) {
            trace.record(slot, outcome, a - b, b - a + CoverageTrace.K);
        }
        return outcome;
    }

    /** {@code a > b}. */
    public boolean gt(int slot, double a, double b) {
        return lt(slot, b, a);
    }

    /** {@code a >= b}. */
    public boolean ge(int slot, double a, double b) {
        return le(slot, b, a);
    }

    /** A boolean condition without numeric operands (distance 1 to the outcome not taken). */
    public boolean flag(int slot, boolean value) {
        if (trace != null) {
            trace.record(slot, value, 1.0, 1.0);
        }
        return value;
    }

    /** A condition whose distances the caller computes (e.g. conjunctions). */
    public boolean record(int slot, boolean outcome, double distanceToTrue, double distanceToFalse) {
        if (trace != null) {
            trace.record(slot, outcome, distanceToTrue, distanceToFalse);
        }
        return outcome;
    }
}
