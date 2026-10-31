package org.robosbse.simulation.controllers;

import org.robosbse.geometry.Angles;
import org.robosbse.simulation.Observation;

/**
 * Waypoint follower whose angular velocity comes from a PID controller on the heading error:
 * {@code omega = kp*e + ki*integral(e) + kd*de/dt}. The integral is clamped (anti-windup) and reset
 * at each waypoint switch; the derivative uses the wrapped difference of successive errors. The
 * gains are the natural target for controller-parameter optimisation (Phase 2).
 */
public final class PidHeadingController extends WaypointFollower {

    private double kp;
    private double ki;
    private double kd;
    private double integralLimit;
    private double dt;
    private double integral;
    private double previousError;
    private boolean first;

    @Override
    public String id() {
        return "pid-heading";
    }

    @Override
    public void reset(ControllerSetup setup) {
        super.reset(setup);
        kp = setup.params().getDouble("kp", 2.0);
        ki = setup.params().getDouble("ki", 0.05);
        kd = setup.params().getDouble("kd", 0.15);
        integralLimit = setup.params().getDouble("integralLimit", 1.0);
        dt = setup.dt();
        integral = 0;
        previousError = 0;
        first = true;
    }

    @Override
    protected void onWaypointSwitch() {
        integral = 0;
        first = true;
    }

    @Override
    protected double headingCommand(double error, Observation obs) {
        integral = Angles.clamp(integral + error * dt, -integralLimit, integralLimit);
        double derivative = first ? 0 : Angles.wrap(error - previousError) / dt;
        first = false;
        previousError = error;
        return Angles.clamp(kp * error + ki * integral + kd * derivative, -maxTurn, maxTurn);
    }
}
