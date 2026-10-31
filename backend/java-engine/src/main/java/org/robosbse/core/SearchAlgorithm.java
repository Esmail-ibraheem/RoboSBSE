package org.robosbse.core;

/**
 * A search algorithm. Implementations loop over {@link SearchContext#evaluate} until it throws
 * {@link SearchTermination}; they may also return normally when they finish by themselves.
 * Algorithms are configured at construction time and must keep no state between runs.
 */
public interface SearchAlgorithm {

    <S> void run(SearchContext<S> context);

    /** Runs the algorithm and packages the bookkeeping of the context as a result. */
    static <S> SearchResult<S> execute(SearchAlgorithm algorithm, SearchContext<S> context) {
        long cpu = SearchContext.currentThreadCpuNanos();
        StopReason reason;
        try {
            algorithm.run(context);
            reason = StopReason.ALGORITHM_FINISHED;
        } catch (SearchTermination termination) {
            reason = termination.reason();
        }
        long used = cpu < 0 ? 0 : SearchContext.currentThreadCpuNanos() - cpu;
        return context.result(reason, used);
    }
}
