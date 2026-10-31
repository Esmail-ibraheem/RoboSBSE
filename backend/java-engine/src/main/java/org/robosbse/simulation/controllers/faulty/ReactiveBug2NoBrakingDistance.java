package org.robosbse.simulation.controllers.faulty;

import org.robosbse.simulation.controllers.ReactiveAvoidanceController;

/**
 * <b>BUG-2 — incorrect stopping-distance threshold (braking distance omitted).</b>
 *
 * <p>The safety check must keep the whole arc the robot sweeps while braking free:
 * {@code v*dt + v^2/(2*decel)}. This variant only checks the distance travelled in the next step,
 * {@code v*dt}, so the robot discovers too late that it cannot stop in time.
 *
 * <p>Expected failure: collision ({@code collision} property). Trigger: situations that force the
 * robot to drive fast towards a nearby obstacle — clutter or a goal placed close to an obstacle —
 * because heading selection normally keeps obstacles about one metre ahead out of the way.
 */
public final class ReactiveBug2NoBrakingDistance extends ReactiveAvoidanceController {

    @Override
    public String id() {
        return "reactive-bug2-braking";
    }

    @Override
    protected double brakingLookahead(double v) {
        return v * dt + 0.02; // BUG: braking distance v^2/(2*decel) missing
    }
}
