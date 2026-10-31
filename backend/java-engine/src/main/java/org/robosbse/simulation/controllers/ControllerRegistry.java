package org.robosbse.simulation.controllers;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.robosbse.core.ParamSpec;
import org.robosbse.simulation.controllers.faulty.ReactiveBug1DropoutAsFree;
import org.robosbse.simulation.controllers.faulty.ReactiveBug2NoBrakingDistance;
import org.robosbse.simulation.controllers.faulty.ReactiveBug3LowBatteryBoost;
import org.robosbse.simulation.controllers.faulty.ReactiveBug4ManhattanGoalCheck;

/** Catalogue of controllers, including the intentionally faulty variants used for testing. */
public final class ControllerRegistry {

    /**
     * @param id               identifier used in configuration
     * @param name             display name
     * @param kind             {@code route-following} (needs waypoints) or {@code reactive} (sensor-based)
     * @param summary          description
     * @param faultyOf         id of the correct controller this is a faulty variant of, or null
     * @param defect           description of the injected defect, or null
     * @param expectedProperty property the defect is expected to violate, or null
     * @param params           tunable parameters
     */
    public record Descriptor(String id, String name, String kind, String summary, String faultyOf, String defect,
                             String expectedProperty, List<ParamSpec> params, Supplier<Controller> factory) {
    }

    private static final List<ParamSpec> FOLLOWER_PARAMS = List.of(
            ParamSpec.real("cruiseSpeed", 1.0, 0.05, 5, "Cruise speed (m/s), capped by the robot limit."),
            ParamSpec.real("kHeading", 2.5, 0, 50, "Proportional heading gain."),
            ParamSpec.real("switchRadius", 0.35, 0.05, 5, "Distance at which the next waypoint is selected (m)."),
            ParamSpec.real("turnInPlaceAngle", 1.0, 0.05, 3.2, "Heading error above which the robot turns in place (rad)."),
            ParamSpec.real("approachGain", 0.8, 0.05, 10, "Final-approach speed gain: v <= gain * distance."));

    private static final List<ParamSpec> REACTIVE_PARAMS = List.of(
            ParamSpec.real("speedLimit", 1.0, 0.05, 5, "Speed limit (m/s)."),
            ParamSpec.real("margin", 0.2, 0, 2, "Safety margin kept around the robot body (m)."),
            ParamSpec.real("kHeading", 2.5, 0, 50, "Proportional heading gain."),
            ParamSpec.real("approachGain", 0.6, 0.05, 10, "Final-approach speed gain."),
            ParamSpec.real("decelerationFraction", 0.8, 0.05, 1, "Assumed braking deceleration as a fraction of the limit."),
            ParamSpec.real("completionFactor", 0.8, 0.05, 1, "Report completion inside this fraction of the goal tolerance."),
            ParamSpec.real("freeDistance", 1.0, 0.1, 10, "Free corridor length required for a heading (m)."),
            ParamSpec.bool("cornerGuard", true, "Add hidden-corner points between adjacent ray hits (false = earlier behaviour)."),
            ParamSpec.integer("pointMemory", 4, 1, 16, "Valid scans whose obstacle points are used together (1 = earlier behaviour)."));

    private static final Map<String, Descriptor> DESCRIPTORS = new LinkedHashMap<>();

    static {
        register(new Descriptor("waypoint-follower", "Waypoint follower", "route-following",
                "Proportional heading control towards successive waypoints; no sensors.",
                null, null, null, FOLLOWER_PARAMS, WaypointFollower::new));
        List<ParamSpec> pid = new ArrayList<>(FOLLOWER_PARAMS);
        pid.add(ParamSpec.real("kp", 2.0, 0, 50, "PID proportional gain."));
        pid.add(ParamSpec.real("ki", 0.05, 0, 50, "PID integral gain."));
        pid.add(ParamSpec.real("kd", 0.15, 0, 50, "PID derivative gain."));
        pid.add(ParamSpec.real("integralLimit", 1.0, 0, 100, "Anti-windup clamp of the integral term."));
        register(new Descriptor("pid-heading", "PID heading controller", "route-following",
                "Waypoint following with a PID controller on the heading error.",
                null, null, null, pid, PidHeadingController::new));
        register(new Descriptor("pure-pursuit", "Pure pursuit path follower", "route-following",
                "Reference path following: steers along the arc to a look-ahead point on the path.",
                null, null, null, List.of(
                        ParamSpec.real("lookahead", 0.6, 0.05, 10, "Look-ahead distance along the path (m)."),
                        ParamSpec.real("cruiseSpeed", 1.0, 0.05, 5, "Cruise speed (m/s)."),
                        ParamSpec.real("approachGain", 0.8, 0.05, 10, "Final-approach speed gain.")),
                PurePursuitController::new));
        register(new Descriptor("reactive", "Reactive obstacle avoidance", "reactive",
                "Map-free goal seeking with range-sensor obstacle avoidance and a braking-distance safety check. "
                        + "Reference (correct) implementation.",
                null, null, null, REACTIVE_PARAMS, ReactiveAvoidanceController::new));
        register(new Descriptor("reactive-bug1-dropout", "Reactive — BUG-1 lost scan = free space", "reactive",
                "Faulty variant: treats a lost sensor scan as an empty world.", "reactive",
                "Incorrect sensor interpretation: lost scans (NaN readings) are treated as free space.",
                "collision", REACTIVE_PARAMS, ReactiveBug1DropoutAsFree::new));
        register(new Descriptor("reactive-bug2-braking", "Reactive — BUG-2 no braking distance", "reactive",
                "Faulty variant: the safety check ignores the braking distance.", "reactive",
                "Incorrect stopping-distance threshold: only v*dt is checked instead of v*dt + v^2/(2a).",
                "collision", REACTIVE_PARAMS, ReactiveBug2NoBrakingDistance::new));
        register(new Descriptor("reactive-bug3-boost", "Reactive — BUG-3 low-battery boost", "reactive",
                "Faulty variant: boosts the speed limit on low battery without clamping.", "reactive",
                "Missing bound check: speed limit * 1.25 below 10% battery is not clamped to the robot maximum.",
                "velocity", REACTIVE_PARAMS, ReactiveBug3LowBatteryBoost::new));
        register(new Descriptor("reactive-bug4-completion", "Reactive — BUG-4 Manhattan goal check", "reactive",
                "Faulty variant: completion check with the Manhattan distance.", "reactive",
                "Wrong distance metric: |dx|+|dy| <= 1.15*tolerance reports completion outside the goal radius.",
                "completion", REACTIVE_PARAMS, ReactiveBug4ManhattanGoalCheck::new));
    }

    private ControllerRegistry() {
    }

    private static void register(Descriptor d) {
        DESCRIPTORS.put(d.id(), d);
    }

    public static Descriptor get(String id) {
        Descriptor d = DESCRIPTORS.get(id);
        if (d == null) {
            throw new IllegalArgumentException("unknown controller '" + id + "', available: " + DESCRIPTORS.keySet());
        }
        return d;
    }

    public static Controller create(String id) {
        return get(id).factory().get();
    }

    public static List<Descriptor> all() {
        return new ArrayList<>(DESCRIPTORS.values());
    }
}
