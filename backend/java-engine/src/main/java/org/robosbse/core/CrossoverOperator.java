package org.robosbse.core;

import java.util.List;

/** Recombines two parents into (usually two) offspring. Implementations never modify inputs. */
@FunctionalInterface
public interface CrossoverOperator<S> {
    List<S> cross(S first, S second, Rng rng);
}
