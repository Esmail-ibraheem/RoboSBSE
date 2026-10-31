package org.robosbse.algorithms.baseline;

import org.robosbse.core.Candidate;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Random walk: moves to a random neighbour at every step regardless of its fitness. It explores the
 * landscape locally without selection pressure — a baseline that isolates the effect of the
 * acceptance rule used by hill climbing and simulated annealing.
 */
public final class RandomWalk implements SearchAlgorithm {

    private final Params params;

    public RandomWalk(Params params) {
        this.params = params;
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        Neighbourhood<S> neighbourhood = context.space().neighbourhood(params);
        Candidate<S> current = context.evaluate(context.space().random(context.rng()));
        while (true) {
            current = context.evaluate(neighbourhood.random(current.solution(), context.rng()));
            context.nextIteration();
        }
    }
}
