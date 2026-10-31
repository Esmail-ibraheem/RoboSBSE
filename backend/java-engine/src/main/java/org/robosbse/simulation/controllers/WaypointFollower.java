package org.robosbse.simulation.controllers;

import java.util.List;

import org.robosbse.geometry.Angles;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;

/**
 * Simple waypoint follower: proportional heading control towards the current waypoint, switching
 * to the next waypoint inside {@code switchRadius}. The robot turns in place when the heading error
 * exceeds {@code turnInPlaceAngle}, otherwise it drives with speed scaled by cos(error) and slows
 * down on the final approach. It has no sensors: obstacle avoidance must come from the route.
 */
public class WaypointFollower implements Controller {

    protected List<Vec2> route;
    protected int index;
    protected double maxSpeed;
    protected double maxTurn;
    protected double kHeading;
    protected double switchRadius;
    protected double turnInPlaceAngle;
    protected double approachGain;

    @Override
    public String id() {
        return "waypoint-follower";
    }

    @Override
    public void reset(ControllerSetup setup) {
        route = setup.route();
        index = 0;
        maxSpeed = Math.min(setup.robot().maxSpeed(), setup.params().getDouble("cruiseSpeed", setup.robot().maxSpeed()));
        maxTurn = setup.robot().maxTurnRate();
        kHeading = setup.params().getDouble("kHeading", 2.5);
        switchRadius = setup.params().getDouble("switchRadius", 0.35);
        turnInPlaceAngle = setup.params().getDouble("turnInPlaceAngle", 1.0);
        approachGain = setup.params().getDouble("approachGain", 0.8);
    }

    /** Advances the waypoint index when the current waypoint is reached; returns the target. */
    protected Vec2 target(Observation obs) {
        while (index < route.size() - 1
                && Math.hypot(route.get(index).x() - obs.x(), route.get(index).y() - obs.y()) < switchRadius) {
            index++;
            onWaypointSwitch();
        }
        return route.get(index);
    }

    protected void onWaypointSwitch() {
    }

    protected double headingCommand(double error, Observation obs) {
        return Angles.clamp(kHeading * error, -maxTurn, maxTurn);
    }

    @Override
    public Command step(Observation obs) {
        Vec2 target = target(obs);
        double dx = target.x() - obs.x();
        double dy = target.y() - obs.y();
        double error = Angles.wrap(Math.atan2(dy, dx) - obs.theta());
        double omega = headingCommand(error, obs);
        double v = Math.abs(error) > turnInPlaceAngle ? 0 : maxSpeed * Math.max(0, Math.cos(error));
        if (index == route.size() - 1) {
            v = Math.min(v, approachGain * Math.hypot(dx, dy) + 0.05);
        }
        return Command.drive(v, omega);
    }
}
