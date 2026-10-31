package org.robosbse.simulation.controllers.faulty;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.simulation.Observation;
import org.robosbse.simulation.controllers.ReactiveAvoidanceController;
import org.robosbse.testing.coverage.Branch;

/**
 * <b>BUG-3 — missing bound check (low-battery speed boost).</b>
 *
 * <p>A "hurry home" feature raises the speed limit by 25% when the battery drops below 10%, but
 * the result is not clamped to the robot's maximum speed. Whenever the configured speed limit is
 * above 80% of the maximum, the controller then commands velocities the hardware cannot deliver.
 *
 * <p>Expected failure: invalid command magnitude ({@code velocity} property). Trigger: initial
 * battery below 0.2 <em>and</em> speed limit above 0.8 m/s <em>and</em> a free straight stretch. The
 * black-box robustness of this property has no gradient towards the trigger (a plateau), which
 * makes it a good example of where white-box (branch-distance) guidance helps: the instrumented
 * decision {@code low-battery-boost} has the branch distance {@code battery - 0.1}.
 */
public final class ReactiveBug3LowBatteryBoost extends ReactiveAvoidanceController {

    private static final int LOW_BATTERY_BOOST = BRANCHES.size();
    private static final List<Branch> BRANCHES_WITH_BOOST;

    static {
        List<Branch> list = new ArrayList<>(BRANCHES);
        list.add(Branch.under("low-battery-boost", "battery < 0.1: raise the speed limit by 25% (the faulty feature)",
                "turn-in-place", false));
        BRANCHES_WITH_BOOST = List.copyOf(list);
    }

    @Override
    public List<Branch> branches() {
        return BRANCHES_WITH_BOOST;
    }

    @Override
    public String instrumentation() {
        return super.instrumentation() + ", plus the low-battery decision of this variant";
    }

    @Override
    public String id() {
        return "reactive-bug3-boost";
    }

    @Override
    protected double speedLimit(Observation obs) {
        if (probe.lt(LOW_BATTERY_BOOST, obs.battery(), 0.1)) {
            return speedLimit * 1.25; // BUG: not clamped to robot.maxSpeed()
        }
        return Math.min(speedLimit, robot.maxSpeed());
    }
}
