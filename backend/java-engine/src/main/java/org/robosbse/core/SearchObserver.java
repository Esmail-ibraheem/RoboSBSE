package org.robosbse.core;

/** Callbacks from a running search. All methods run on the search thread and should be cheap. */
public interface SearchObserver<S> {

    default void onEvaluation(SearchContext<S> context, Candidate<S> candidate) {
    }

    default void onNewBest(SearchContext<S> context, Candidate<S> best) {
    }

    default void onTarget(SearchContext<S> context, Candidate<S> target) {
    }

    static <S> SearchObserver<S> none() {
        return new SearchObserver<>() {
        };
    }
}
