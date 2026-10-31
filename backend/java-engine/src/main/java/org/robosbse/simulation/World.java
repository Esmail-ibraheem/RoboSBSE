package org.robosbse.simulation;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;

/**
 * Runtime view of a {@link WorldMap} with fast geometric queries: clearance, collision and ray
 * casting. Immutable and therefore safe to share between threads.
 *
 * <p>The time-free queries see the static obstacles only (planners and occupancy grids use them). The
 * queries with a time argument add the moving and timed obstacles at that time; they compute the static
 * part first, exactly as the time-free query does, so a map without dynamic elements gives identical
 * results either way.
 */
public final class World {

    private final WorldMap map;
    private final Shape[] obstacles;
    private final double[] boundX;
    private final double[] boundY;
    private final double[] boundR;
    private final MovingObstacle[] moving;
    private final TimedObstacle[] timed;
    private final Zone[] terrain;
    private final Zone[] restricted;

    public World(WorldMap map) {
        this.map = map;
        this.obstacles = map.obstacles().toArray(new Shape[0]);
        int n = obstacles.length;
        boundX = new double[n];
        boundY = new double[n];
        boundR = new double[n];
        for (int i = 0; i < n; i++) {
            Vec2 c = obstacles[i].boundingCentre();
            boundX[i] = c.x();
            boundY[i] = c.y();
            boundR[i] = obstacles[i].boundingRadius();
        }
        this.moving = map.movingObstacles().toArray(new MovingObstacle[0]);
        this.timed = map.timedObstacles().toArray(new TimedObstacle[0]);
        this.terrain = map.zones().stream().filter(z -> !z.restricted()).toArray(Zone[]::new);
        this.restricted = map.zones().stream().filter(Zone::restricted).toArray(Zone[]::new);
    }

    /** Whether obstacles change over time. */
    public boolean dynamic() {
        return moving.length > 0 || timed.length > 0;
    }

    /** Whether the map has restricted zones. */
    public boolean hasRestrictedZones() {
        return restricted.length > 0;
    }

    /** Signed distance to the nearest obstacle at time {@code t}, moving and timed obstacles included. */
    public double obstacleDistance(double x, double y, double t) {
        double best = obstacleDistance(x, y);
        if (moving.length == 0 && timed.length == 0) {
            return best;
        }
        return Math.min(best, dynamicDistance(x, y, t));
    }

    /** Signed distance to the nearest moving or timed obstacle at time {@code t} (infinite if none). */
    public double dynamicDistance(double x, double y, double t) {
        double best = Double.POSITIVE_INFINITY;
        for (MovingObstacle m : moving) {
            Vec2 p = m.position(t);
            best = Math.min(best, Math.hypot(x - p.x(), y - p.y()) - m.radius());
        }
        for (TimedObstacle o : timed) {
            if (o.active(t)) {
                best = Math.min(best, o.shape().signedDistance(x, y));
            }
        }
        return best;
    }

    /** Signed distance to the nearest obstacle or border at time {@code t}. */
    public double distance(double x, double y, double t) {
        return Math.min(obstacleDistance(x, y, t), borderDistance(x, y));
    }

    /** Ray cast at time {@code t}, seeing moving and timed obstacles. */
    public double raycast(double ox, double oy, double angle, double maxRange, double t) {
        double best = raycast(ox, oy, angle, maxRange);
        if (moving.length == 0 && timed.length == 0) {
            return best;
        }
        double dx = Math.cos(angle);
        double dy = Math.sin(angle);
        for (MovingObstacle m : moving) {
            Vec2 p = m.position(t);
            best = Math.min(best, new Circle(p.x(), p.y(), m.radius()).raycast(ox, oy, dx, dy));
        }
        for (TimedObstacle o : timed) {
            if (o.active(t)) {
                best = Math.min(best, o.shape().raycast(ox, oy, dx, dy));
            }
        }
        return best;
    }

    /** Terrain speed factor at the point: the smallest factor of the terrain zones containing it, else 1. */
    public double speedFactor(double x, double y) {
        double f = 1.0;
        for (Zone z : terrain) {
            if (z.shape().signedDistance(x, y) <= 0) {
                f = Math.min(f, z.speedFactor());
            }
        }
        return f;
    }

    /** Whether a disc of the given radius overlaps a restricted zone. */
    public boolean inRestrictedZone(double x, double y, double radius) {
        for (Zone z : restricted) {
            if (z.shape().signedDistance(x, y) < radius) {
                return true;
            }
        }
        return false;
    }

    public WorldMap map() {
        return map;
    }

    public double width() {
        return map.width();
    }

    public double height() {
        return map.height();
    }

    public List<Shape> obstacles() {
        return List.of(obstacles);
    }

    /** World with additional obstacles (used by generated test scenarios). */
    public World withExtraObstacles(List<Shape> extra) {
        List<Shape> all = new ArrayList<>(map.obstacles());
        all.addAll(extra);
        return new World(map.withObstacles(all));
    }

    /** Signed distance from the point to the nearest obstacle surface (ignores the border). */
    public double obstacleDistance(double x, double y) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < obstacles.length; i++) {
            // Bounding-circle lower bound lets us skip shapes that cannot be the nearest.
            double lower = Math.hypot(x - boundX[i], y - boundY[i]) - boundR[i];
            if (lower >= best) {
                continue;
            }
            best = Math.min(best, obstacles[i].signedDistance(x, y));
        }
        return best;
    }

    /** Distance from the point to the world border (negative outside the world). */
    public double borderDistance(double x, double y) {
        return Math.min(Math.min(x, map.width() - x), Math.min(y, map.height() - y));
    }

    /** Signed distance to the nearest obstacle or border. */
    public double distance(double x, double y) {
        return Math.min(obstacleDistance(x, y), borderDistance(x, y));
    }

    /** True if a disc of the given radius overlaps an obstacle or leaves the world. */
    public boolean collides(double x, double y, double radius) {
        return distance(x, y) < radius;
    }

    /**
     * Casts a ray and returns the distance to the first obstacle or border hit, capped at
     * {@code maxRange}.
     */
    public double raycast(double ox, double oy, double angle, double maxRange) {
        double dx = Math.cos(angle);
        double dy = Math.sin(angle);
        double best = maxRange;
        // Border: the ray starts inside the world box and leaves it at the nearest wall.
        if (dx > 1e-12) {
            best = Math.min(best, (map.width() - ox) / dx);
        } else if (dx < -1e-12) {
            best = Math.min(best, -ox / dx);
        }
        if (dy > 1e-12) {
            best = Math.min(best, (map.height() - oy) / dy);
        } else if (dy < -1e-12) {
            best = Math.min(best, -oy / dy);
        }
        best = Math.max(0, best);
        for (int i = 0; i < obstacles.length; i++) {
            double toCentre = Math.hypot(ox - boundX[i], oy - boundY[i]);
            if (toCentre - boundR[i] >= best) {
                continue;
            }
            double t = obstacles[i].raycast(ox, oy, dx, dy);
            if (t < best) {
                best = t;
            }
        }
        return best;
    }
}
