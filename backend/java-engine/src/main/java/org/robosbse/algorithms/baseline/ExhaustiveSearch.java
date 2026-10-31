package org.robosbse.algorithms.baseline;

import java.util.Iterator;

import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Exhaustive (brute-force) search: evaluates every solution of a finite, enumerable space in a
 * fixed order. If the enumeration completes within the budget the best solution is a proven
 * optimum (stat {@code complete = 1}); otherwise the run is an ordinary truncated scan. Only
 * sensible for small instances, e.g. scheduling 4 tasks (8! = 40 320 orders).
 */
public final class ExhaustiveSearch implements SearchAlgorithm {

    @Override
    public <S> void run(SearchContext<S> context) {
        if (!context.space().enumerable()) {
            throw new IllegalArgumentException("exhaustive search needs an enumerable representation, not '"
                    + context.space().representation() + "'");
        }
        Iterator<S> all = context.space().enumerate();
        context.stat("complete", 0);
        while (all.hasNext()) {
            context.evaluate(all.next());
            context.nextIteration();
        }
        context.stat("complete", 1);
    }
}
