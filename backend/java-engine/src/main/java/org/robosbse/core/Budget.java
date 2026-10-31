package org.robosbse.core;

/**
 * Search budget. The primary, algorithm-independent budget is the number of fitness evaluations,
 * which is what makes comparisons between algorithms fair. An optional wall-clock limit guards
 * against unexpectedly slow runs; it excludes time spent paused.
 *
 * @param maxEvaluations maximum number of fitness evaluations (must be positive)
 * @param maxMillis      wall-clock limit in milliseconds, or 0 for none
 */
public record Budget(long maxEvaluations, long maxMillis) {

    public Budget {
        if (maxEvaluations <= 0) {
            throw new IllegalArgumentException("maxEvaluations must be positive");
        }
        if (maxMillis < 0) {
            throw new IllegalArgumentException("maxMillis must not be negative");
        }
    }

    public static Budget evaluations(long maxEvaluations) {
        return new Budget(maxEvaluations, 0);
    }
}
