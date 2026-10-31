package org.robosbse.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.robosbse.core.Params;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Vec2;
import org.robosbse.problems.NavigationProblem;
import org.robosbse.protocol.Json;

class DynamicWorldTest {

    private static Vec2 v(double x, double y) {
        return new Vec2(x, y);
    }

    @Test
    void movingObstaclesBounceAndLoopAlongTheirPaths() {
        MovingObstacle bounce = new MovingObstacle(0.5, List.of(v(0, 0), v(4, 0)), 1, "bounce", 0, null);
        assertEquals(v(3, 0), bounce.position(3));
        assertEquals(v(3, 0), bounce.position(5), "turns around at the end of the path");
        assertEquals(v(0, 0), bounce.position(8));
        MovingObstacle loop = new MovingObstacle(0.4, List.of(v(0, 0), v(2, 0), v(2, 2)), 1, "loop", 1, null);
        Vec2 p = loop.position(2);
        assertEquals(2, p.x(), 1e-12);
        assertEquals(1, p.y(), 1e-12);
        assertEquals(4 + Math.sqrt(8), loop.length(), 1e-12);
        MovingObstacle parked = new MovingObstacle(0.3, List.of(v(5, 5)), 2, "loop", 0, null);
        assertEquals(v(5, 5), parked.position(17));
    }

    @Test
    void timedObstaclesExistOnlyInTheirWindow() {
        TimedObstacle pallet = new TimedObstacle(new Rect(1, 1, 1, 1), 15, 40.0, "pallet");
        assertFalse(pallet.active(14.99));
        assertTrue(pallet.active(15));
        assertFalse(pallet.active(40));
        assertTrue(new TimedObstacle(new Rect(1, 1, 1, 1), 5, null, null).active(1e6), "no end time: stays");
    }

    @Test
    void worldDistancesFollowTimeAndZones() {
        WorldMap base = MapLibrary.builtin("empty-field");
        WorldMap map = new WorldMap(base.schema(), "d", null, base.width(), base.height(), base.start(), base.goal(),
                base.goalTolerance(), base.obstacles(),
                List.of(new MovingObstacle(0.5, List.of(v(5, 5), v(9, 5)), 1, "bounce", 0, "guard")),
                List.of(new TimedObstacle(new Circle(3, 3, 0.5), 10, 20.0, "box")),
                List.of(new Zone("terrain", new Rect(10, 10, 2, 2), 0.5, "gravel"),
                        new Zone("restricted", new Rect(14, 14, 2, 2), 0, "keep out")));
        World world = new World(map);
        assertTrue(world.dynamic());
        assertTrue(world.hasRestrictedZones());
        assertEquals(-0.5, world.obstacleDistance(5, 5, 0), 1e-9, "guard centre at t = 0");
        assertEquals(1.5, world.obstacleDistance(7, 5, 0), 1e-9);
        assertEquals(-0.5, world.obstacleDistance(7, 5, 2), 1e-9, "guard moved 2 m");
        assertTrue(world.obstacleDistance(3, 3, 5) > 0, "box not there yet");
        assertEquals(-0.5, world.obstacleDistance(3, 3, 15), 1e-9);
        assertEquals(0.5, world.speedFactor(11, 11), 1e-12);
        assertEquals(1.0, world.speedFactor(5, 15), 1e-12);
        assertTrue(world.inRestrictedZone(15, 15, 0.25));
        assertTrue(world.inRestrictedZone(13.9, 15, 0.25), "the body overlapping the zone counts");
        assertFalse(world.inRestrictedZone(13.5, 15, 0.25));
        WorldMap again = Json.convert(Json.tree(map), WorldMap.class);
        assertEquals(map, again, "dynamic elements survive the JSON round trip");
    }

    @Test
    void staticMapsAreUnchangedByTheTimeAwareQueries() {
        World world = new World(MapLibrary.builtin("obstacle-course"));
        assertFalse(world.dynamic());
        for (double x = 0.3; x < world.width(); x += 1.37) {
            for (double y = 0.3; y < world.height(); y += 1.11) {
                assertEquals(world.obstacleDistance(x, y), world.obstacleDistance(x, y, 42.0), 0.0);
                assertEquals(world.distance(x, y), world.distance(x, y, 7.0), 0.0);
                assertEquals(world.raycast(x, y, 0.7, 4), world.raycast(x, y, 0.7, 4, 3.0), 0.0);
            }
        }
    }

    @Test
    @SuppressWarnings("unchecked")
    void staticAStarRouteIsHitByTheGuardsOfTheDynamicCourse() {
        NavigationProblem p = new NavigationProblem(MapLibrary.builtin("dynamic-obstacle-course"), Params.empty(),
                Params.empty(), RobotSpec.DEFAULT);
        Map<String, Object> astar = (Map<String, Object>) p.reference().get("astar");
        assertEquals(Boolean.TRUE, astar.get("found"));
        Map<String, Double> metrics = (Map<String, Double>) astar.get("metrics");
        assertEquals(1.0, metrics.get("dynamicCollision"), "a planner that ignores time collides with a moving obstacle");
    }
}
