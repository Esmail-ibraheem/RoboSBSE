package subjects;

import org.robosbse.geometry.Angles;
import org.robosbse.patching.WorkCounter;
import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * <b>Missing boundary check</b> (repair subject RB4): The speed clamp only triggers above 1.5 times the robot's maximum speed, so the low-battery boost (+25%) commands speeds the hardware cannot deliver.
 *
 * <p>Generated from the RoboSBSE subject template: map-free reactive navigation (heading selection over
 * free corridors, obstacle points of the last four scans, hidden-corner points between adjacent ray
 * hits, braking-distance speed limiter, low-battery speed boost clamped to the robot's maximum).
 * 
 */
public class BoundaryCheckBug implements Controller, WorkCounter {

    private static final int HEADING_CANDIDATES = 37;
    private static final int ARC_SAMPLES = 6;
    private static final int MEMORY_SCANS = 4;
    private static final double STEERING_SIGN = 1.0;

    private RobotSpec robot;
    private double dt;
    private double goalX;
    private double goalY;
    private double goalTolerance;
    private double speedLimit;
    private double margin;
    private double kHeading;
    private double approachGain;
    private double deceleration;
    private double completionFactor;
    private double freeDistance;
    private double[] pointsX = new double[0];
    private double[] pointsY = new double[0];
    private int pointCount;
    private double[] rayX = new double[0];
    private double[] rayY = new double[0];
    private boolean[] rayHit = new boolean[0];
    private double[][] memX = new double[0][];
    private double[][] memY = new double[0][];
    private int[] memCount = new int[0];
    private int memHead;
    private int memFilled;
    private double previousHeading = Double.NaN;
    private double[] fx = new double[0];
    private double[] fy = new double[0];
    private long work;

    @Override
    public String id() {
        return "subject-rb4-boundary-check";
    }

    @Override
    public boolean usesSensors() {
        return true;
    }

    @Override
    public long work() {
        return work;
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
        rayX = new double[robot.sensorRays()];
        rayY = new double[robot.sensorRays()];
        rayHit = new boolean[robot.sensorRays()];
        memX = new double[MEMORY_SCANS][2 * robot.sensorRays()];
        memY = new double[MEMORY_SCANS][2 * robot.sensorRays()];
        memCount = new int[MEMORY_SCANS];
        memHead = 0;
        memFilled = 0;
        pointsX = new double[MEMORY_SCANS * 2 * robot.sensorRays()];
        pointsY = new double[MEMORY_SCANS * 2 * robot.sensorRays()];
        fx = new double[MEMORY_SCANS * 2 * robot.sensorRays()];
        fy = new double[MEMORY_SCANS * 2 * robot.sensorRays()];
        pointCount = 0;
        previousHeading = Double.NaN;
        work = 0;
    }

    @Override
    public Command step(Observation obs) {
        if (obs.scanValid()) {
            pointCount = 0;
            for (int i = 0; i < obs.rays(); i++) {
                double r = obs.range(i);
                double a = obs.theta() + robot.rayAngle(i);
                rayX[i] = obs.x() + r * Math.cos(a);
                rayY[i] = obs.y() + r * Math.sin(a);
                rayHit[i] = false;
                if (r < robot.sensorRange() - 1e-6) {
                    rayHit[i] = true;
                    pointsX[pointCount] = rayX[i];
                    pointsY[pointCount] = rayY[i];
                    pointCount++;
                }
            }
            addCornerPoints(obs);
            remember();
        }
        double dx = goalX - obs.x();
        double dy = goalY - obs.y();
        if (goalReached(dx, dy)) {
            return Command.complete();
        }
        double distanceToGoal = Math.hypot(dx, dy);
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
            if (free > escapeFree) {
                escapeFree = free;
                escape = heading;
            }
            if (free >= needed) {
                double cost = Math.abs(offset);
                if (!Double.isNaN(previousHeading)) {
                    cost += 0.5 * Math.abs(Angles.wrap(heading - previousHeading));
                }
                if (cost < chosenCost) {
                    chosenCost = cost;
                    chosen = heading;
                }
            }
        }
        if (Double.isNaN(chosen)) {
            chosen = escape;
        }
        previousHeading = chosen;
        double error = Angles.wrap(chosen - obs.theta());
        double omega = Angles.clamp(STEERING_SIGN * kHeading * error, -robot.maxTurnRate(), robot.maxTurnRate());
        double vDesired = Math.abs(error) > 1.2 ? 0 : speedLimit(obs) * Math.max(0, Math.cos(error));
        vDesired = Math.min(vDesired, approachGain * distanceToGoal + 0.03);
        double v = safeSpeed(obs, vDesired, omega);
        return Command.drive(v, omega);
    }

    private void addCornerPoints(Observation obs) {
        double gap = 1.5 * (robot.radius() + margin);
        int pairs = obs.rays();
        for (int i = 0; i < pairs; i++) {
            int j = (i + 1) % pairs;
            double chord = Math.hypot(rayX[j] - rayX[i], rayY[j] - rayY[i]);
            if (rayHit[i] && rayHit[j] && chord <= gap) {
                double mx = (rayX[i] + rayX[j]) / 2;
                double my = (rayY[i] + rayY[j]) / 2;
                double d = Math.hypot(mx - obs.x(), my - obs.y());
                pointsX[pointCount] = mx - chord / 2 * (mx - obs.x()) / d;
                pointsY[pointCount] = my - chord / 2 * (my - obs.y()) / d;
                pointCount++;
            }
        }
    }

    private void remember() {
        System.arraycopy(pointsX, 0, memX[memHead], 0, pointCount);
        System.arraycopy(pointsY, 0, memY[memHead], 0, pointCount);
        memCount[memHead] = pointCount;
        memHead = (memHead + 1) % MEMORY_SCANS;
        memFilled = Math.min(memFilled + 1, MEMORY_SCANS);
        pointCount = 0;
        for (int k = 0; k < memFilled; k++) {
            System.arraycopy(memX[k], 0, pointsX, pointCount, memCount[k]);
            System.arraycopy(memY[k], 0, pointsY, pointCount, memCount[k]);
            pointCount += memCount[k];
        }
    }

    private boolean goalReached(double dx, double dy) {
        return Math.hypot(dx, dy) <= completionFactor * goalTolerance;
    }

    private double speedLimit(Observation obs) {
        double limit = speedLimit;
        if (obs.battery() < 0.1) {
            limit = limit * 1.25;
        }
        if (limit > robot.maxSpeed() * 1.5) {
            limit = robot.maxSpeed();
        }
        return limit;
    }

    private double brakingLookahead(double v) {
        return v * dt + v * v / (2.0 * deceleration) + 0.02;
    }

    private double corridorFree(double x, double y, double heading) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double halfWidth = robot.radius() + margin;
        double free = robot.sensorRange();
        for (int i = 0; i < pointCount; i++) {
            work++;
            double dxw = pointsX[i] - x;
            double dyw = pointsY[i] - y;
            double along = dxw * c + dyw * s;
            double lateral = -dxw * s + dyw * c;
            if (along > 0 && Math.abs(lateral) < halfWidth) {
                free = Math.min(free, along - Math.sqrt(halfWidth * halfWidth - lateral * lateral));
            }
        }
        return free;
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
        while (v >= 0.03) {
            if (arcClear(v, omega)) {
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
            if (Math.abs(kappa) < 1e-6) {
                sx = arc;
                sy = 0;
            } else {
                sx = Math.sin(kappa * arc) / kappa;
                sy = (1 - Math.cos(kappa * arc)) / kappa;
            }
            for (int i = 0; i < pointCount; i++) {
                work++;
                double now = Math.hypot(fx[i], fy[i]);
                double after = Math.hypot(fx[i] - sx, fy[i] - sy);
                if (after < Math.min(required, now) - 0.005) {
                    return false;
                }
            }
        }
        return true;
    }
}
