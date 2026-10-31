package org.robosbse.core;

import java.util.ArrayList;
import java.util.List;

/**
 * Neighbourhood structure for local search.
 *
 * <p>{@link #all} enumerates a finite neighbourhood (needed by first- and best-improvement hill
 * climbing); {@link #random} samples a single neighbour (stochastic hill climbing, simulated
 * annealing, random walk); {@link #moves} additionally describes each neighbour by a <em>move
 * attribute</em> and the attribute of the move that would undo it (tabu search).
 */
public interface Neighbourhood<S> {

    List<S> all(S solution);

    S random(S solution, Rng rng);

    /**
     * The neighbourhood as moves. The default is solution-based: the attribute of a move is the
     * neighbour itself and its inverse is the current solution, so tabu search forbids returning to
     * recently visited solutions. Representations override this with attribute-based moves.
     */
    default List<Move<S>> moves(S solution) {
        List<Move<S>> moves = new ArrayList<>();
        for (S neighbour : all(solution)) {
            moves.add(new Move<>(neighbour, neighbour, solution));
        }
        return moves;
    }
}
