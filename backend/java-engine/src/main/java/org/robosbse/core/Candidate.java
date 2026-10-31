package org.robosbse.core;

/**
 * An evaluated solution.
 *
 * @param solution   the (possibly repaired) solution that was evaluated
 * @param evaluation its evaluation
 * @param index      1-based number of the fitness evaluation that produced it within the run
 */
public record Candidate<S>(S solution, Evaluation evaluation, long index) {

    public double fitness() {
        return evaluation.fitness();
    }

    /** Strictly better under minimisation. */
    public boolean betterThan(Candidate<?> other) {
        return other == null || fitness() < other.fitness();
    }
}
