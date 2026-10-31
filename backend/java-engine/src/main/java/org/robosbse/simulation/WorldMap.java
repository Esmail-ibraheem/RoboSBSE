package org.robosbse.simulation;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonInclude;

import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;

/**
 * Serialisable environment description (the {@code robosbse.map/1} file format shared with the UI).
 *
 * <p>Coordinates are metres; the world spans [0, width] x [0, height] and its border behaves like a
 * wall. Obstacles are static circles and axis-aligned rectangles. Phase 5 adds optional dynamic
 * elements: {@link MovingObstacle}s, {@link TimedObstacle}s (environmental events) and {@link Zone}s
 * (restricted areas, slow terrain). They are omitted from the JSON when empty, so static maps keep their
 * Phase 1 form; unknown fields are ignored.
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonInclude(JsonInclude.Include.NON_NULL)
public record WorldMap(String schema, String name, String description, double width, double height,
                       Pose start, Vec2 goal, double goalTolerance, List<Shape> obstacles,
                       @JsonInclude(JsonInclude.Include.NON_EMPTY) List<MovingObstacle> movingObstacles,
                       @JsonInclude(JsonInclude.Include.NON_EMPTY) List<TimedObstacle> timedObstacles,
                       @JsonInclude(JsonInclude.Include.NON_EMPTY) List<Zone> zones) {

    public static final String SCHEMA = "robosbse.map/1";

    public WorldMap {
        if (!(width > 0) || !(height > 0)) {
            throw new IllegalArgumentException("map width and height must be positive");
        }
        schema = schema == null ? SCHEMA : schema;
        name = name == null ? "unnamed" : name;
        obstacles = obstacles == null ? List.of() : List.copyOf(obstacles);
        movingObstacles = movingObstacles == null ? List.of() : List.copyOf(movingObstacles);
        timedObstacles = timedObstacles == null ? List.of() : List.copyOf(timedObstacles);
        zones = zones == null ? List.of() : List.copyOf(zones);
        goalTolerance = goalTolerance > 0 ? goalTolerance : 0.3;
        if (start == null) {
            start = new Pose(1, 1, 0);
        }
        if (goal == null) {
            goal = new Vec2(width - 1, height - 1);
        }
    }

    /** A static map (no moving or timed obstacles, no zones). */
    public WorldMap(String schema, String name, String description, double width, double height, Pose start, Vec2 goal,
                    double goalTolerance, List<Shape> obstacles) {
        this(schema, name, description, width, height, start, goal, goalTolerance, obstacles, null, null, null);
    }

    public WorldMap withObstacles(List<Shape> newObstacles) {
        return new WorldMap(schema, name, description, width, height, start, goal, goalTolerance, newObstacles,
                movingObstacles, timedObstacles, zones);
    }

    public WorldMap withStartAndGoal(Pose newStart, Vec2 newGoal) {
        return new WorldMap(schema, name, description, width, height, newStart, newGoal, goalTolerance, obstacles,
                movingObstacles, timedObstacles, zones);
    }

    /** Whether obstacles change over time (moving or timed obstacles). */
    public boolean dynamic() {
        return !movingObstacles.isEmpty() || !timedObstacles.isEmpty();
    }
}
