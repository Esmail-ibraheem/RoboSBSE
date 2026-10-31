package org.robosbse.core;

import java.util.Iterator;
import java.util.List;

/**
 * Candidate representation together with the operators that search algorithms need.
 *
 * <p>Operators are created from algorithm parameters, so the same representation can offer several
 * mutation, crossover or neighbourhood variants (selected by name in the experiment configuration;
 * {@code "default"} selects the representation's first operator). An algorithm only uses the
 * operators it needs: random search only {@link #random}, hill climbing a {@link Neighbourhood},
 * a genetic algorithm mutation and crossover.
 */
public interface SolutionSpace<S> {

    /** Representation name, e.g. {@code real-vector} or {@code permutation}. */
    String representation();

    /** Uniformly (or otherwise documented) random solution. */
    S random(Rng rng);

    /** Mutation operator configured from algorithm parameters. */
    MutationOperator<S> mutation(Params params);

    /** Crossover operator configured from algorithm parameters. */
    CrossoverOperator<S> crossover(Params params);

    /** Neighbourhood used by local search, configured from algorithm parameters. */
    Neighbourhood<S> neighbourhood(Params params);

    /** Genotypic distance used for diversity measurement (normalised to roughly [0, 1]). */
    double distance(S a, S b);

    /** Names of the available mutation operators (first is the default). */
    List<String> mutationOperators();

    /** Names of the available crossover operators (first is the default). */
    List<String> crossoverOperators();

    /** Names of the available neighbourhoods (first is the default). */
    default List<String> neighbourhoods() {
        return List.of("default");
    }

    /** Number of decision variables, for documentation and default rates. */
    int dimension();

    /** Whether {@link #enumerate()} can list the whole space (exhaustive search). */
    default boolean enumerable() {
        return false;
    }

    /** Iterates over every solution of a finite space. */
    default Iterator<S> enumerate() {
        throw new UnsupportedOperationException(representation() + " spaces cannot be enumerated");
    }

    /**
     * Resolves an operator parameter: absent or {@code "default"} means the first available
     * operator; unknown names are rejected with the list of valid ones.
     */
    static String operator(Params params, String key, List<String> available, String representation) {
        String name = params.getString(key, "default");
        if (name.equals("default")) {
            return available.get(0);
        }
        if (!available.contains(name)) {
            throw new IllegalArgumentException(key + " '" + name + "' is not available for the " + representation
                    + " representation; available: " + available);
        }
        return name;
    }
}
