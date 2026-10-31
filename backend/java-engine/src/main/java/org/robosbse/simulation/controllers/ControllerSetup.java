package org.robosbse.simulation.controllers;

import java.util.List;

import org.robosbse.core.Params;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.RobotSpec;

/**
 * Per-episode information handed to a controller.
 *
 * @param robot         physical robot parameters
 * @param dt            control period (s)
 * @param goal          final destination
 * @param goalTolerance radius around the goal that counts as arrived (m)
 * @param route         reference waypoints (ending at the goal) for path-following controllers;
 *                      reactive controllers ignore it
 * @param params        controller parameters (gains, thresholds, speed limit)
 */
public record ControllerSetup(RobotSpec robot, double dt, Vec2 goal, double goalTolerance, List<Vec2> route,
                              Params params) {

    public ControllerSetup {
        route = route == null || route.isEmpty() ? List.of(goal) : List.copyOf(route);
        params = params == null ? Params.empty() : params;
    }
}
