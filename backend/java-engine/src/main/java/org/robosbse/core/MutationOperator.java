package org.robosbse.core;

/** Produces a modified copy of a solution. Implementations never modify their input. */
@FunctionalInterface
public interface MutationOperator<S> {
    S mutate(S parent, Rng rng);
}
