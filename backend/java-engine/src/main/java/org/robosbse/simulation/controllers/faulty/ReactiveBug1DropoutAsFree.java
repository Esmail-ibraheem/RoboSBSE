package org.robosbse.simulation.controllers.faulty;

import org.robosbse.simulation.Observation;
import org.robosbse.simulation.controllers.ReactiveAvoidanceController;

/**
 * <b>BUG-1 — incorrect sensor interpretation (lost scan treated as free space).</b>
 *
 * <p>When a range scan is lost the sensor driver reports every reading as {@code NaN}. The correct
 * controller keeps the obstacle points of the last valid scan (obstacles are static). This variant
 * discards them, so for the duration of a dropout streak the robot believes the world is empty:
 * heading selection aims straight at the goal and the braking check passes at full speed.
 *
 * <p>Expected failure: collision ({@code collision} property). Trigger: a high scan-dropout
 * probability while an obstacle lies close to the robot between it and the goal.
 */
public final class ReactiveBug1DropoutAsFree extends ReactiveAvoidanceController {

    @Override
    public String id() {
        return "reactive-bug1-dropout";
    }

    @Override
    protected void onScanLost(Observation obs) {
        pointCount = 0; // BUG: "no readings" interpreted as "no obstacles"
    }
}
