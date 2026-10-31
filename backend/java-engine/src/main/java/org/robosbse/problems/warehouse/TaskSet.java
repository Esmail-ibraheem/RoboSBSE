package org.robosbse.problems.warehouse;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.geometry.Vec2;

/**
 * A warehouse order book ({@code robosbse.tasks/1}): pickup-and-delivery tasks on a map, plus the
 * robot's mission parameters.
 *
 * @param map            map reference ({@code "builtin:warehouse"}, a file or an inline map)
 * @param depot          where the robot starts and must return
 * @param charger        charging station
 * @param capacity       items the robot can carry at once
 * @param speed          planning speed (m/s) used by the travel-time model
 * @param serviceTime    seconds spent at each pickup or delivery
 * @param chargeRate     charging power (J/s)
 * @param initialBattery initial charge as a fraction of the robot's capacity
 * @param reserve        charge fraction that must always remain
 * @param tasks          the orders
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record TaskSet(String schema, String name, String description, JsonNode map, Vec2 depot, Vec2 charger,
                      int capacity, double speed, double serviceTime, double chargeRate, double initialBattery,
                      double reserve, List<Task> tasks) {

    public static final String SCHEMA = "robosbse.tasks/1";

    /**
     * One order: carry an item from {@code pickup} to {@code delivery}, ideally before
     * {@code deadline} (seconds after the mission starts), and only after every task listed in
     * {@code after} has been delivered.
     */
    @JsonIgnoreProperties(ignoreUnknown = true)
    public record Task(String id, Vec2 pickup, Vec2 delivery, double deadline, List<String> after) {
        public Task {
            after = after == null ? List.of() : List.copyOf(after);
        }
    }

    public TaskSet {
        tasks = tasks == null ? List.of() : List.copyOf(tasks);
        capacity = capacity <= 0 ? 1 : capacity;
        speed = speed > 0 ? speed : 0.8;
        chargeRate = chargeRate > 0 ? chargeRate : 50;
        initialBattery = initialBattery > 0 ? initialBattery : 1.0;
    }
}
