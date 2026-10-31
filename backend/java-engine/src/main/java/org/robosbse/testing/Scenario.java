package org.robosbse.testing;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.Pose;

/**
 * A concrete test scenario (the phenotype of a scenario genome): the inputs and environmental
 * conditions under which the controller is executed.
 *
 * @param start       robot start pose
 * @param goal        destination given to the controller
 * @param sensorNoise standard deviation of range noise (m)
 * @param scanDropout probability of losing a whole scan per step
 * @param battery     initial battery charge (fraction)
 * @param speedLimit  controller speed-limit configuration (m/s)
 * @param freeDistance controller configuration: free corridor length required for a heading (m)
 * @param obstacles   obstacles added to the base arena
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record Scenario(Pose start, Vec2 goal, double sensorNoise, double scanDropout, double battery,
                       double speedLimit, double freeDistance, List<Shape> obstacles) {

    public Scenario {
        obstacles = obstacles == null ? List.of() : List.copyOf(obstacles);
    }

    public Scenario withObstacles(List<Shape> newObstacles) {
        return new Scenario(start, goal, sensorNoise, scanDropout, battery, speedLimit, freeDistance, newObstacles);
    }

    public Scenario withSensing(double noise, double dropout) {
        return new Scenario(start, goal, noise, dropout, battery, speedLimit, freeDistance, obstacles);
    }

    public Scenario withBattery(double charge) {
        return new Scenario(start, goal, sensorNoise, scanDropout, charge, speedLimit, freeDistance, obstacles);
    }

    public Scenario withSpeedLimit(double limit) {
        return new Scenario(start, goal, sensorNoise, scanDropout, battery, limit, freeDistance, obstacles);
    }
}
