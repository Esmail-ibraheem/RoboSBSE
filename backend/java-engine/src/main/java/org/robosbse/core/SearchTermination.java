package org.robosbse.core;

/**
 * Thrown by {@link SearchContext#evaluate} to unwind an algorithm when the run must stop. Algorithms
 * do not need to check the budget themselves: they simply keep requesting evaluations, which keeps
 * their code close to the textbook pseudo-code and guarantees nobody exceeds the budget.
 */
public final class SearchTermination extends RuntimeException {

    private final StopReason reason;

    public SearchTermination(StopReason reason) {
        super(reason.name(), null, false, false);
        this.reason = reason;
    }

    public StopReason reason() {
        return reason;
    }
}
