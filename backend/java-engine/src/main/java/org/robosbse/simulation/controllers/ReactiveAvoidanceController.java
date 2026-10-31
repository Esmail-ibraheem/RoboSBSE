package org.robosbse.simulation.controllers;

import java.util.List;

import org.robosbse.geometry.Angles;
import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.testing.coverage.Branch;
import org.robosbse.testing.coverage.CoverageTrace;
import org.robosbse.testing.coverage.Instrumented;
import org.robosbse.testing.coverage.Probe;

/**
 * Sensor-based reactive controller: drives to the goal using only the range sensor (no map), and is
 * the main <em>system under test</em> of the search-based testing module.
 *
 * <p>Per step it
 * <ol>
 *   <li><b>perceives</b>: converts valid range readings into world-frame obstacle points. Because
 *       Phase 1 obstacles are static, the points of the last valid scan stay correct, so a lost scan
 *       is handled by reusing them;</li>
 *   <li><b>checks completion</b>: reports the task done when the Euclidean distance to the goal is
 *       below {@code completionFactor * goalTolerance};</li>
 *   <li><b>selects a heading</b>: among 37 candidate directions around the goal bearing it picks the
 *       one closest to the goal (with hysteresis) whose straight corridor of half-width
 *       {@code radius + margin} is free for {@code min(distanceToGoal, freeDistance)} metres;</li>
 *   <li><b>limits the speed for safety</b>: the commanded speed is reduced until the arc the robot
 *       would sweep while braking to a stop (length {@code v*dt + v^2 / (2*decel)}) keeps every
 *       obstacle point at least {@code radius + margin} away (or no closer than it already is).
 *       Turning in place is always allowed — it cannot cause a collision for a disc robot.</li>
 * </ol>
 *
 * <p>The protected hook methods are the injection points for the documented faulty variants in
 * {@code org.robosbse.simulation.controllers.faulty}; each variant overrides exactly one hook.
 * Known limitation (not a defect): as any purely reactive controller it can get trapped in
 * U-shaped obstacle configurations and cannot pass gaps narrower than {@code 2 * (radius + margin)}.
 *
 * <p><b>Coverage instrumentation.</b> Every explicit conditional of this class ({@code if},
 * {@code while}, {@code ?:}) is wrapped in a {@link Probe} call that returns the condition's value
 * unchanged and, when a trace is attached, records the outcome and its branch distances
 * ({@link #BRANCHES}). Library calls such as {@code Math.min} and the fixed-count {@code for} loops are
 * not counted as decisions.
 */
public class ReactiveAvoidanceController implements Controller, Instrumented {

    private static final int HEADING_CANDIDATES = 37;
    private static final int ARC_SAMPLES = 6;

    // Probe slots: indices into BRANCHES.
    protected static final int SCAN_VALID = 0;
    protected static final int RAY_HIT = 1;
    protected static final int GOAL_REACHED = 2;
    protected static final int ESCAPE_IMPROVES = 3;
    protected static final int HEADING_FREE = 4;
    protected static final int HAS_PREVIOUS_HEADING = 5;
    protected static final int BETTER_HEADING = 6;
    protected static final int NO_FREE_HEADING = 7;
    protected static final int TURN_IN_PLACE = 8;
    protected static final int IN_CORRIDOR = 9;
    protected static final int SPEED_LOOP = 10;
    protected static final int ARC_CLEAR = 11;
    protected static final int STRAIGHT_ARC = 12;
    protected static final int ARC_BLOCKED = 13;

    /**
     * The instrumented decisions. A parent is the decision outcome under which the decision is
     * evaluated (its control dependency, or the loop-entry condition for decisions inside the loops
     * over obstacle points).
     */
    public static final List<Branch> BRANCHES = List.of(
            Branch.root("scan-valid", "obs.scanValid(): a range scan arrived (else the last obstacle points are reused)"),
            Branch.under("ray-hit", "r < sensorRange: a ray returned an obstacle within range", "scan-valid", true),
            Branch.root("goal-reached", "goalReached(dx, dy): |goal - position| <= completionFactor * goalTolerance"),
            Branch.under("escape-improves", "free > escapeFree: candidate heading has the longest free corridor so far",
                    "goal-reached", false),
            Branch.under("heading-free", "free >= needed: candidate corridor is free for min(distance to goal, freeDistance)",
                    "goal-reached", false),
            Branch.under("has-previous-heading", "!isNaN(previousHeading): a heading was chosen before (hysteresis term)",
                    "heading-free", true),
            Branch.under("better-heading", "cost < chosenCost: candidate is cheaper than the best free heading so far",
                    "heading-free", true),
            Branch.under("no-free-heading", "isNaN(chosen): no candidate corridor is free, the escape heading is used",
                    "goal-reached", false),
            Branch.under("turn-in-place", "|heading error| > 1.2 rad: turn on the spot (desired speed 0)",
                    "goal-reached", false),
            Branch.under("in-corridor", "along > 0 && |lateral| < radius + margin: an obstacle point lies in a candidate corridor",
                    "ray-hit", true),
            Branch.under("speed-loop", "v >= 0.03: the safety limiter still tries a non-zero speed", "goal-reached", false),
            Branch.under("arc-clear", "arcClear(v, omega): the braking arc at speed v keeps every point at a safe distance",
                    "speed-loop", true),
            Branch.under("straight-arc", "|omega / v| < 1e-6: the braking arc is a straight segment", "speed-loop", true),
            Branch.under("arc-blocked", "after < min(required, now) - 0.005: an arc sample comes too close to a point",
                    "speed-loop", true));

    /** Coverage probe; records nothing unless a trace is attached. */
    protected final Probe probe = new Probe();

    protected RobotSpec robot;
    protected double dt;
    protected double goalX;
    protected double goalY;
    protected double goalTolerance;
    protected double speedLimit;
    protected double margin;
    protected double kHeading;
    protected double approachGain;
    protected double deceleration;
    protected double completionFactor;
    protected double freeDistance;

    /** Obstacle points in world coordinates used for the current decision. */
    protected double[] pointsX = new double[0];
    protected double[] pointsY = new double[0];
    protected int pointCount;
    private double previousHeading = Double.NaN;

    // Robot-frame copies, recomputed every step.
    private double[] fx = new double[0];
    private double[] fy = new double[0];

    @Override
    public String id() {
        return "reactive";
    }

    @Override
    public boolean usesSensors() {
        return true;
    }

    @Override
    public List<Branch> branches() {
        return BRANCHES;
    }

    @Override
    public void attach(CoverageTrace trace) {
        probe.attach(trace);
    }

    @Override
    public String instrumentation() {
        return "hand-placed probes on all " + BRANCHES.size()
                + " explicit conditionals (if, while, ?:) of ReactiveAvoidanceController";
    }

    @Override
    public void reset(ControllerSetup setup) {
        robot = setup.robot();
        dt = setup.dt();
        goalX = setup.goal().x();
        goalY = setup.goal().y();
        goalTolerance = setup.goalTolerance();
        speedLimit = setup.params().getDouble("speedLimit", robot.maxSpeed());
        margin = setup.params().getDouble("margin", 0.2);
        kHeading = setup.params().getDouble("kHeading", 2.5);
        approachGain = setup.params().getDouble("approachGain", 0.6);
        deceleration = setup.params().getDouble("decelerationFraction", 0.8) * robot.maxAcceleration();
        completionFactor = setup.params().getDouble("completionFactor", 0.8);
        freeDistance = setup.params().getDouble("freeDistance", 1.0);
        pointsX = new double[robot.sensorRays()];
        pointsY = new double[robot.sensorRays()];
        fx = new double[robot.sensorRays()];
        fy = new double[robot.sensorRays()];
        pointCount = 0;
        previousHeading = Double.NaN;
    }

    @Override
    public Command step(Observation obs) {
        if (probe.flag(SCAN_VALID, obs.scanValid())) {
            pointCount = 0;
            for (int i = 0; i < obs.rays(); i++) {
                double r = obs.range(i);
                if (probe.lt(RAY_HIT, r, robot.sensorRange() - 1e-6)) {
                    double a = obs.theta() + robot.rayAngle(i);
                    pointsX[pointCount] = obs.x() + r * Math.cos(a);
                    pointsY[pointCount] = obs.y() + r * Math.sin(a);
                    pointCount++;
                }
            }
        } else {
            onScanLost(obs);
        }
        double dx = goalX - obs.x();
        double dy = goalY - obs.y();
        if (goalReached(dx, dy)) {
            return Command.complete();
        }
        double distanceToGoal = Math.hypot(dx, dy);

        // Points relative to the robot centre (world-aligned) for heading selection.
        double needed = Math.min(distanceToGoal, freeDistance);
        double bearing = Math.atan2(dy, dx);
        double chosen = Double.NaN;
        double chosenCost = Double.POSITIVE_INFINITY;
        double escape = bearing;
        double escapeFree = Double.NEGATIVE_INFINITY;
        for (int k = 0; k < HEADING_CANDIDATES; k++) {
            int offsetIndex = (k + 1) / 2 * (k % 2 == 0 ? 1 : -1);
            double offset = Math.toRadians(10.0 * offsetIndex);
            double heading = bearing + offset;
            double free = corridorFree(obs.x(), obs.y(), heading);
            if (probe.gt(ESCAPE_IMPROVES, free, escapeFree)) {
                escapeFree = free;
                escape = heading;
            }
            if (probe.ge(HEADING_FREE, free, needed)) {
                double cost = Math.abs(offset);
                if (probe.flag(HAS_PREVIOUS_HEADING, !Double.isNaN(previousHeading))) {
                    cost += 0.5 * Math.abs(Angles.wrap(heading - previousHeading));
                }
                if (probe.lt(BETTER_HEADING, cost, chosenCost)) {
                    chosenCost = cost;
                    chosen = heading;
                }
            }
        }
        // No heading was chosen exactly when even the freest corridor is shorter than needed.
        if (probe.record(NO_FREE_HEADING, Double.isNaN(chosen), escapeFree - needed + CoverageTrace.K, needed - escapeFree)) {
            chosen = escape;
        }
        previousHeading = chosen;

        double error = Angles.wrap(chosen - obs.theta());
        double omega = Angles.clamp(kHeading * error, -robot.maxTurnRate(), robot.maxTurnRate());
        double vDesired = probe.gt(TURN_IN_PLACE, Math.abs(error), 1.2) ? 0 : speedLimit(obs) * Math.max(0, Math.cos(error));
        vDesired = Math.min(vDesired, approachGain * distanceToGoal + 0.03);
        double v = safeSpeed(obs, vDesired, omega);
        return Command.drive(v, omega);
    }

    /** Hook: perception when the scan is lost. Correct behaviour keeps the last valid points. */
    protected void onScanLost(Observation obs) {
        // Static obstacles: the previous points remain valid in world coordinates.
    }

    /** Hook: completion check (Euclidean distance against a tolerance safely inside the goal radius). */
    protected boolean goalReached(double dx, double dy) {
        return probe.le(GOAL_REACHED, Math.hypot(dx, dy), completionFactor * goalTolerance);
    }

    /** Hook: speed limit for the current situation. */
    protected double speedLimit(Observation obs) {
        return Math.min(speedLimit, robot.maxSpeed());
    }

    /** Hook: length of the arc that must be free — travel in one step plus the braking distance. */
    protected double brakingLookahead(double v) {
        return v * dt + v * v / (2 * deceleration) + 0.02;
    }

    /** Free length of the straight corridor (half-width radius + margin) along a world heading. */
    private double corridorFree(double x, double y, double heading) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double halfWidth = robot.radius() + margin;
        double free = robot.sensorRange();
        for (int i = 0; i < pointCount; i++) {
            double dxw = pointsX[i] - x;
            double dyw = pointsY[i] - y;
            double along = dxw * c + dyw * s;
            double lateral = -dxw * s + dyw * c;
            if (inCorridor(along, lateral, halfWidth)) {
                free = Math.min(free, along - Math.sqrt(halfWidth * halfWidth - lateral * lateral));
            }
        }
        return free;
    }

    /** {@code along > 0 && |lateral| < halfWidth}, with conjunction branch distances when traced. */
    private boolean inCorridor(double along, double lateral, double halfWidth) {
        boolean inside = along > 0 && Math.abs(lateral) < halfWidth;
        if (probe.trace() == null) {
            return inside;
        }
        double lateralGap = Math.abs(lateral) - halfWidth;
        double toTrue = (along > 0 ? 0 : -along + CoverageTrace.K) + (lateralGap < 0 ? 0 : lateralGap + CoverageTrace.K);
        double toFalse = Math.min(Math.max(0, along), Math.max(0, -lateralGap));
        return probe.record(IN_CORRIDOR, inside, toTrue, toFalse);
    }

    private double safeSpeed(Observation obs, double vDesired, double omega) {
        double c = Math.cos(obs.theta());
        double s = Math.sin(obs.theta());
        for (int i = 0; i < pointCount; i++) {
            double dxw = pointsX[i] - obs.x();
            double dyw = pointsY[i] - obs.y();
            fx[i] = dxw * c + dyw * s;
            fy[i] = -dxw * s + dyw * c;
        }
        double v = vDesired;
        while (probe.ge(SPEED_LOOP, v, 0.03)) {
            if (probe.flag(ARC_CLEAR, arcClear(v, omega))) {
                return v;
            }
            v *= 0.6;
        }
        return 0;
    }

    private boolean arcClear(double v, double omega) {
        double length = brakingLookahead(v);
        double kappa = omega / v;
        double required = robot.radius() + margin;
        for (int j = 1; j <= ARC_SAMPLES; j++) {
            double arc = length * j / ARC_SAMPLES;
            double sx;
            double sy;
            if (probe.lt(STRAIGHT_ARC, Math.abs(kappa), 1e-6)) {
                sx = arc;
                sy = 0;
            } else {
                sx = Math.sin(kappa * arc) / kappa;
                sy = (1 - Math.cos(kappa * arc)) / kappa;
            }
            for (int i = 0; i < pointCount; i++) {
                double now = Math.hypot(fx[i], fy[i]);
                double after = Math.hypot(fx[i] - sx, fy[i] - sy);
                if (probe.lt(ARC_BLOCKED, after, Math.min(required, now) - 0.005)) {
                    return false;
                }
            }
        }
        return true;
    }
}
