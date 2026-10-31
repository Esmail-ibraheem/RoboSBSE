package org.robosbse.core;

/** Why a search run ended. */
public enum StopReason {
    /** The fitness-evaluation budget was used up. */
    BUDGET_EXHAUSTED,
    /** The wall-clock limit was reached. */
    TIME_LIMIT,
    /** A target evaluation (e.g. a discovered failure) was reached and stop-on-target was set. */
    TARGET_REACHED,
    /** The algorithm terminated by itself (e.g. hill climbing without restarts at a local optimum). */
    ALGORITHM_FINISHED,
    /** The user cancelled the run. */
    CANCELLED
}
