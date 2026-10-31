package org.robosbse.core;

import java.util.List;

/**
 * An optimisation problem: a solution space plus a deterministic fitness function.
 *
 * <p>Implementations must be <em>pure</em> with respect to {@link #evaluate}: the same solution
 * always yields the same evaluation (stochastic simulation effects are driven by a fixed,
 * problem-level seed — "common random numbers"). This makes runs reproducible and lets algorithms
 * be compared fairly.
 *
 * <p>Multi-objective problems additionally name their objectives ({@link #objectiveNames()}); the
 * evaluation then carries an objective vector (all minimised) next to the scalar fitness, which
 * single-objective algorithms keep using.
 *
 * @param <S> solution (genotype) type
 */
public interface Problem<S> {

    /** Stable identifier of the problem type, e.g. {@code "navigation"}. */
    String type();

    /** Representation and search operators for candidates of this problem. */
    SolutionSpace<S> space();

    /** Deterministic fitness evaluation (minimised). */
    Evaluation evaluate(S solution);

    /**
     * Optional repair applied before evaluation. The repaired solution replaces the original
     * (Lamarckian repair). The default is the identity.
     */
    default S repair(S solution) {
        return solution;
    }

    /**
     * Whether an evaluation reaches the problem's target (e.g. a safety violation was found, or a
     * known optimum was hit). Target hits are recorded by the {@link SearchContext}.
     */
    default boolean isTarget(Evaluation evaluation) {
        return false;
    }

    /** A JSON-serialisable view of a solution for reports and the UI. */
    default Object view(S solution) {
        return solution;
    }

    /** Names of the minimised objectives, empty for single-objective problems. */
    default List<String> objectiveNames() {
        return List.of();
    }

    /**
     * Reference point for the hypervolume indicator (one coordinate per objective, worse than any
     * solution of interest), or null when the problem defines none.
     */
    default double[] referencePoint() {
        return null;
    }

    /**
     * Whether {@link #repair} and {@link #evaluate} may run concurrently on several threads. The
     * bundled problems are immutable after construction and evaluate each candidate in fresh
     * simulator and controller instances, so the default is true.
     */
    default boolean parallelSafe() {
        return true;
    }

    /**
     * Optional problem-specific neighbourhood that memetic algorithms can use for local improvement
     * (e.g. route smoothing for navigation), or null when the problem has none.
     */
    default Neighbourhood<S> domainNeighbourhood() {
        return null;
    }
}
