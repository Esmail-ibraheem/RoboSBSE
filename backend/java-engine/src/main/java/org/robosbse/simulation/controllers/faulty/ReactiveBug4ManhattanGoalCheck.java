package org.robosbse.simulation.controllers.faulty;

import org.robosbse.simulation.controllers.ReactiveAvoidanceController;

/**
 * <b>BUG-4 — wrong distance metric in the completion check.</b>
 *
 * <p>To "avoid a square root" the completion check uses the Manhattan distance
 * {@code |dx| + |dy| <= 1.15 * tolerance}. The diamond-shaped acceptance region pokes outside the
 * circular goal region near the coordinate axes, so the controller reports completion while still
 * outside the goal tolerance when it approaches the goal within roughly 9 degrees of an axis
 * direction.
 *
 * <p>Expected failure: false task completion ({@code completion} property).
 */
public final class ReactiveBug4ManhattanGoalCheck extends ReactiveAvoidanceController {

    @Override
    public String id() {
        return "reactive-bug4-completion";
    }

    @Override
    protected boolean goalReached(double dx, double dy) {
        return probe.le(GOAL_REACHED, Math.abs(dx) + Math.abs(dy), 1.15 * goalTolerance); // BUG: L1 instead of Euclidean
    }
}
