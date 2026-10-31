package org.robosbse.algorithms.baseline;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Uniform random search: repeatedly samples independent solutions from the representation's
 * {@code random} generator and keeps the best (the context tracks it). It uses no information from
 * previous evaluations and is the baseline any metaheuristic must beat to justify itself.
 * Samples are evaluated in batches (in parallel when the context has several threads); the result
 * is the same as evaluating them one by one.
 */
public final class RandomSearch implements SearchAlgorithm {

    private static final int BATCH = 64;

    @Override
    public <S> void run(SearchContext<S> context) {
        while (true) {
            int size = context.threads() > 1 ? BATCH : 1;
            List<S> batch = new ArrayList<>(size);
            for (int i = 0; i < size; i++) {
                batch.add(context.space().random(context.rng()));
            }
            for (Candidate<S> ignored : context.evaluateAll(batch)) {
                context.nextIteration();
            }
        }
    }
}
