package org.robosbse.simulation;

import org.robosbse.core.Rng;
import org.robosbse.geometry.Angles;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Deterministic fixed-step simulator of a disc-shaped unicycle robot among static obstacles.
 *
 * <p>Each step: (1) sense — cast the range rays, apply Gaussian noise and scan dropout; (2) ask the
 * controller for a command and validate it; (3) saturate the command to the actuator limits and
 * apply acceleration limits; (4) integrate the unicycle model with the midpoint rule; (5) account
 * energy; (6) check collision, border, goal, battery and deadlock conditions.
 *
 * <p>Dynamic maps: sensing and collision checks use the moving and timed obstacles at the current time,
 * terrain zones scale the achievable speed, and the time the body overlaps a restricted zone is measured.
 * On static maps these steps reduce exactly to the static computation.
 *
 * <p>Sensor randomness uses a dedicated stream seeded by {@link EpisodeConfig#noiseSeed()}, and the
 * same number of random draws happens every step regardless of the scenario. Two scenarios that
 * differ only in noise level or dropout probability therefore see the <em>same</em> underlying
 * random numbers ("common random numbers"), which makes testing fitness landscapes smooth in those
 * variables and keeps every episode exactly reproducible.
 */
public final class Simulator {

    private static final double EPS = 1e-9;
    private static final int PATH_RESOLUTION = 400;

    private Simulator() {
    }

    public static EpisodeResult run(World world, RobotSpec robot, Pose start, Vec2 goal, double goalTolerance,
                                    Controller controller, ControllerSetup setup, EpisodeConfig cfg) {
        double dt = cfg.dt();
        double x = start.x();
        double y = start.y();
        double theta = Angles.wrap(start.theta());
        double v = 0;
        double omega = 0;
        double energyLeft = robot.batteryCapacity() * cfg.initialBattery();
        double energyUsed = 0;
        double distance = 0;
        double smoothness = 0;
        int collisions = 0;
        boolean inContact = false;
        double minObstacle = cap(world.obstacleDistance(x, y, 0) - robot.radius());
        boolean dynamicCollision = false;
        int restrictedSteps = 0;
        double minBorder = world.borderDistance(x, y) - robot.radius();
        double maxCmdSpeed = 0;
        double maxCmdTurn = 0;
        int boundViolations = 0;
        boolean reportedDone = false;
        double reportedDistance = -1;
        int dropped = 0;
        String error = null;
        Outcome outcome = null;
        int step = 0;
        boolean lastScanValid = true;

        boolean sense = controller.usesSensors();
        int rays = robot.sensorRays();
        Rng noise = new Rng(cfg.noiseSeed());
        double[] ranges = sense ? new double[rays] : null;

        int windowSteps = cfg.deadlockWindow() > 0 ? (int) Math.round(cfg.deadlockWindow() / dt) : 0;
        double[] histX = windowSteps > 0 ? new double[windowSteps] : null;
        double[] histY = windowSteps > 0 ? new double[windowSteps] : null;

        Trajectory trajectory = cfg.record() ? new Trajectory(dt * cfg.recordEvery(), cfg.recordRays() && sense) : null;
        // Coarse position history for behaviour descriptors (path shape at fixed fractions of the episode).
        int stride = Math.max(1, cfg.maxSteps() / PATH_RESOLUTION);
        double[] path = new double[2 * (cfg.maxSteps() / stride + 2)];
        int pathLength = 0;
        controller.reset(setup);

        for (step = 0; step < cfg.maxSteps(); step++) {
            double time = step * dt;
            boolean scanValid = true;
            if (sense) {
                // Fixed number of draws per step: one for dropout, one per ray for noise.
                double dropDraw = noise.nextDouble();
                scanValid = dropDraw >= cfg.scanDropout();
                for (int i = 0; i < rays; i++) {
                    double z = noise.gaussian();
                    double r = world.raycast(x, y, theta + robot.rayAngle(i), robot.sensorRange(), time);
                    if (r < robot.sensorRange()) {
                        r = Angles.clamp(r + cfg.sensorNoise() * z, 0, robot.sensorRange());
                    }
                    ranges[i] = scanValid ? r : Double.NaN;
                }
                if (!scanValid) {
                    dropped++;
                }
                lastScanValid = scanValid;
            }
            if (step % stride == 0) {
                path[2 * pathLength] = x;
                path[2 * pathLength + 1] = y;
                pathLength++;
            }
            double battery = robot.batteryCapacity() > 0 ? energyLeft / robot.batteryCapacity() : 0;
            if (trajectory != null && step % cfg.recordEvery() == 0) {
                trajectory.add(time, x, y, theta, v, omega, battery,
                        currentClearance(world, x, y, robot.radius(), time), scanValid, ranges);
            }

            Observation obs = new Observation(time, x, y, theta, v, omega, battery, scanValid,
                    ranges == null ? null : ranges.clone());
            Command cmd;
            try {
                cmd = controller.step(obs);
            } catch (RuntimeException e) {
                error = e.getClass().getSimpleName() + ": " + e.getMessage();
                outcome = Outcome.CONTROLLER_ERROR;
                break;
            }
            if (cmd == null || !cmd.finite()) {
                outcome = Outcome.INVALID_COMMAND;
                break;
            }
            maxCmdSpeed = Math.max(maxCmdSpeed, cmd.v());
            maxCmdTurn = Math.max(maxCmdTurn, Math.abs(cmd.omega()));
            if (cmd.v() > robot.maxSpeed() + EPS || cmd.v() < -robot.maxReverseSpeed() - EPS
                    || Math.abs(cmd.omega()) > robot.maxTurnRate() + EPS) {
                boundViolations++;
            }
            double toGoal = Math.hypot(goal.x() - x, goal.y() - y);
            if (cmd.done()) {
                reportedDone = true;
                reportedDistance = toGoal;
                outcome = toGoal <= goalTolerance ? Outcome.REACHED_GOAL : Outcome.FALSE_COMPLETION;
                break;
            }

            // Actuator saturation and acceleration limits.
            double terrain = world.speedFactor(x, y);
            double vTarget = Angles.clamp(cmd.v(), -robot.maxReverseSpeed() * terrain, robot.maxSpeed() * terrain);
            double wTarget = Angles.clamp(cmd.omega(), -robot.maxTurnRate(), robot.maxTurnRate());
            double dvMax = robot.maxAcceleration() * dt;
            double dwMax = robot.maxAngularAcceleration() * dt;
            double newV = v + Angles.clamp(vTarget - v, -dvMax, dvMax);
            double newW = omega + Angles.clamp(wTarget - omega, -dwMax, dwMax);
            double acceleration = (newV - v) / dt;
            smoothness += Math.abs(newW - omega);
            v = newV;
            omega = newW;

            // Midpoint integration of the unicycle model.
            double prevX = x;
            double prevY = y;
            double midTheta = theta + omega * dt / 2;
            x += v * Math.cos(midTheta) * dt;
            y += v * Math.sin(midTheta) * dt;
            theta = Angles.wrap(theta + omega * dt);
            distance += Math.abs(v) * dt;

            double power = robot.power(v, omega, acceleration);
            energyUsed += power * dt;
            energyLeft -= power * dt;

            double obstacleClear = cap(world.obstacleDistance(x, y, (step + 1) * dt) - robot.radius());
            if (world.hasRestrictedZones() && world.inRestrictedZone(x, y, robot.radius())) {
                restrictedSteps++;
            }
            double borderClear = world.borderDistance(x, y) - robot.radius();
            minObstacle = Math.min(minObstacle, obstacleClear);
            minBorder = Math.min(minBorder, borderClear);
            if (obstacleClear < 0 || borderClear < 0) {
                if (!inContact) {
                    collisions++;
                }
                if (obstacleClear < 0 && world.dynamic() && world.dynamicDistance(x, y, (step + 1) * dt) - robot.radius() < 0) {
                    dynamicCollision = true;
                }
                inContact = true;
                if (cfg.stopOnCollision()) {
                    outcome = obstacleClear < 0 ? Outcome.COLLISION : Outcome.OUT_OF_BOUNDS;
                    step++;
                    break;
                }
                // Blocked: undo the motion and stop.
                x = prevX;
                y = prevY;
                v = 0;
            } else {
                inContact = false;
            }

            if (cfg.autoComplete() && Math.hypot(goal.x() - x, goal.y() - y) <= goalTolerance) {
                outcome = Outcome.REACHED_GOAL;
                step++;
                break;
            }
            if (energyLeft <= 0) {
                outcome = Outcome.BATTERY_DEPLETED;
                step++;
                break;
            }
            if (windowSteps > 0) {
                int slot = step % windowSteps;
                if (step >= windowSteps && Math.hypot(x - histX[slot], y - histY[slot]) < cfg.deadlockDistance()) {
                    outcome = Outcome.DEADLOCK;
                    step++;
                    break;
                }
                histX[slot] = x;
                histY[slot] = y;
            }
        }
        if (outcome == null) {
            outcome = Outcome.TIMEOUT;
        }
        if (trajectory != null) {
            trajectory.add(step * dt, x, y, theta, v, omega,
                    robot.batteryCapacity() > 0 ? Math.max(0, energyLeft) / robot.batteryCapacity() : 0,
                    currentClearance(world, x, y, robot.radius(), step * dt), lastScanValid, sense ? ranges : null);
        }
        return new EpisodeResult(outcome, step, step * dt, distance, energyUsed, collisions, minObstacle,
                minBorder, smoothness, new Pose(x, y, theta), Math.hypot(goal.x() - x, goal.y() - y),
                maxCmdSpeed, maxCmdTurn, boundViolations, reportedDone, reportedDistance, dropped, error,
                restrictedSteps * dt, dynamicCollision, pathSamples(path, pathLength, stride, step, x, y), trajectory);
    }

    /** Positions at 25%, 50% and 75% of the episode's duration, from the coarse position history. */
    private static double[] pathSamples(double[] path, int length, int stride, int steps, double x, double y) {
        double[] out = new double[6];
        for (int i = 0; i < 3; i++) {
            int k = Math.min(length - 1, (int) ((i + 1) * 0.25 * steps / stride));
            out[2 * i] = k >= 0 ? path[2 * k] : x;
            out[2 * i + 1] = k >= 0 ? path[2 * k + 1] : y;
        }
        return out;
    }

    /** Clearance values are capped so that obstacle-free worlds still produce finite numbers. */
    private static double cap(double clearance) {
        return Math.min(clearance, 1e3);
    }

    private static double currentClearance(World world, double x, double y, double radius, double time) {
        return cap(world.distance(x, y, time) - radius);
    }
}
