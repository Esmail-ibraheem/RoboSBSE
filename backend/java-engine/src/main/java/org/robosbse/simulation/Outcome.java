package org.robosbse.simulation;

/** How an episode terminated. */
public enum Outcome {
    /** The robot is within the goal tolerance (auto-completion or a correct completion report). */
    REACHED_GOAL,
    /** The robot body penetrated an obstacle. */
    COLLISION,
    /** The robot body crossed the world border. */
    OUT_OF_BOUNDS,
    /** The step limit was reached. */
    TIMEOUT,
    /** The robot made no progress for the deadlock window. */
    DEADLOCK,
    /** The battery ran empty before the task finished. */
    BATTERY_DEPLETED,
    /** The controller returned a non-finite command (or null). */
    INVALID_COMMAND,
    /** The controller threw an exception. */
    CONTROLLER_ERROR,
    /** The controller reported completion while outside the goal tolerance. */
    FALSE_COMPLETION;

    public boolean success() {
        return this == REACHED_GOAL;
    }
}
