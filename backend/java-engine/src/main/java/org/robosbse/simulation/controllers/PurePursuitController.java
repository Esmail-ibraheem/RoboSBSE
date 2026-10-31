package org.robosbse.simulation.controllers;

import java.util.List;

import org.robosbse.geometry.Angles;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;

/**
 * Reference path follower using pure pursuit. The reference path is the polyline from the start
 * position through the route waypoints. Each step the controller projects the robot onto the path
 * (searching forward only, so progress is monotone), picks the point {@code lookahead} metres
 * further along the path and steers on the circular arc through it: curvature
 * {@code kappa = 2 sin(alpha) / L}, {@code omega = v * kappa}.
 */
public final class PurePursuitController implements Controller {

    private double[] px;
    private double[] py;
    private double[] cumulative;
    private int segment;
    private double lookahead;
    private double maxSpeed;
    private double maxTurn;
    private double approachGain;
    private boolean started;
    private Vec2 goal;

    @Override
    public String id() {
        return "pure-pursuit";
    }

    @Override
    public void reset(ControllerSetup setup) {
        lookahead = setup.params().getDouble("lookahead", 0.6);
        maxSpeed = Math.min(setup.robot().maxSpeed(), setup.params().getDouble("cruiseSpeed", setup.robot().maxSpeed()));
        maxTurn = setup.robot().maxTurnRate();
        approachGain = setup.params().getDouble("approachGain", 0.8);
        goal = setup.goal();
        List<Vec2> route = setup.route();
        px = new double[route.size() + 1];
        py = new double[route.size() + 1];
        for (int i = 0; i < route.size(); i++) {
            px[i + 1] = route.get(i).x();
            py[i + 1] = route.get(i).y();
        }
        started = false;
        segment = 0;
    }

    private void initialise(Observation obs) {
        px[0] = obs.x();
        py[0] = obs.y();
        cumulative = new double[px.length];
        for (int i = 1; i < px.length; i++) {
            cumulative[i] = cumulative[i - 1] + Math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]);
        }
        started = true;
    }

    @Override
    public Command step(Observation obs) {
        if (!started) {
            initialise(obs);
        }
        // Project onto the path, looking at most a few segments ahead.
        double bestDist = Double.POSITIVE_INFINITY;
        double bestS = cumulative[segment];
        int bestSeg = segment;
        for (int i = segment; i < Math.min(px.length - 1, segment + 4); i++) {
            double sx = px[i + 1] - px[i];
            double sy = py[i + 1] - py[i];
            double len2 = sx * sx + sy * sy;
            double u = len2 > 0 ? Angles.clamp(((obs.x() - px[i]) * sx + (obs.y() - py[i]) * sy) / len2, 0, 1) : 0;
            double qx = px[i] + u * sx;
            double qy = py[i] + u * sy;
            double d = Math.hypot(obs.x() - qx, obs.y() - qy);
            if (d < bestDist - 1e-9) {
                bestDist = d;
                bestSeg = i;
                bestS = cumulative[i] + u * Math.sqrt(len2);
            }
        }
        segment = bestSeg;
        Vec2 target = pointAt(bestS + lookahead);
        double dx = target.x() - obs.x();
        double dy = target.y() - obs.y();
        double alpha = Angles.wrap(Math.atan2(dy, dx) - obs.theta());
        double remaining = cumulative[cumulative.length - 1] - bestS + bestDist;
        double v = Math.abs(alpha) > Math.PI / 2 ? 0 : maxSpeed * Math.max(0.2, Math.cos(alpha));
        v = Math.min(v, approachGain * Math.max(remaining, goal.distanceTo(obs.x(), obs.y())) + 0.05);
        double distanceToTarget = Math.max(1e-6, Math.hypot(dx, dy));
        double omega = v > 1e-6
                ? v * 2 * Math.sin(alpha) / distanceToTarget
                : Math.signum(alpha) * maxTurn;
        return Command.drive(v, Angles.clamp(omega, -maxTurn, maxTurn));
    }

    private Vec2 pointAt(double s) {
        double total = cumulative[cumulative.length - 1];
        if (s >= total) {
            return new Vec2(px[px.length - 1], py[py.length - 1]);
        }
        for (int i = 0; i < px.length - 1; i++) {
            if (s <= cumulative[i + 1]) {
                double segLen = cumulative[i + 1] - cumulative[i];
                double u = segLen > 0 ? (s - cumulative[i]) / segLen : 0;
                return new Vec2(px[i] + u * (px[i + 1] - px[i]), py[i] + u * (py[i + 1] - py[i]));
            }
        }
        return new Vec2(px[px.length - 1], py[py.length - 1]);
    }
}
