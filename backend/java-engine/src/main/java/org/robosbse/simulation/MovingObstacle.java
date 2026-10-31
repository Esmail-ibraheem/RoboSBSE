package org.robosbse.simulation;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import org.robosbse.geometry.Vec2;

/**
 * A circular obstacle that moves along a polyline at constant speed, independent of the robot (it does
 * not react to it). Its position is a pure function of time, so episodes stay deterministic.
 *
 * @param radius obstacle radius (m)
 * @param path   at least one point; a single point means a stationary obstacle
 * @param speed  speed along the path (m/s)
 * @param mode   {@code bounce}: back and forth along the path; {@code loop}: around the closed polygon
 * @param offset distance along the path at time 0 (m)
 * @param label  optional name
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record MovingObstacle(double radius, List<Vec2> path, double speed, String mode, double offset, String label) {

    public MovingObstacle {
        if (!(radius > 0)) {
            throw new IllegalArgumentException("moving obstacle radius must be positive");
        }
        if (path == null || path.isEmpty()) {
            throw new IllegalArgumentException("a moving obstacle needs a path with at least one point");
        }
        path = List.copyOf(path);
        mode = mode == null ? "bounce" : mode;
        if (!mode.equals("bounce") && !mode.equals("loop")) {
            throw new IllegalArgumentException("moving obstacle mode must be 'bounce' or 'loop'");
        }
    }

    /** Length of one traversal: the open polyline (bounce) or the closed polygon (loop). */
    public double length() {
        double l = 0;
        for (int i = 1; i < path.size(); i++) {
            l += path.get(i - 1).distanceTo(path.get(i));
        }
        if (mode.equals("loop") && path.size() > 1) {
            l += path.get(path.size() - 1).distanceTo(path.get(0));
        }
        return l;
    }

    /** Centre at time {@code t} (seconds). */
    public Vec2 position(double t) {
        double length = length();
        if (path.size() == 1 || length <= 0 || speed == 0) {
            return at(offset);
        }
        double s = speed * t + offset;
        if (mode.equals("loop")) {
            s = mod(s, length);
        } else {
            s = mod(s, 2 * length);
            if (s > length) {
                s = 2 * length - s;
            }
        }
        return at(s);
    }

    private Vec2 at(double distance) {
        double remaining = Math.max(0, distance);
        int n = path.size();
        int segments = mode.equals("loop") ? n : n - 1;
        for (int i = 0; i < segments; i++) {
            Vec2 a = path.get(i);
            Vec2 b = path.get((i + 1) % n);
            double l = a.distanceTo(b);
            if (remaining <= l || i == segments - 1) {
                double f = l <= 0 ? 0 : Math.min(1, remaining / l);
                return new Vec2(a.x() + f * (b.x() - a.x()), a.y() + f * (b.y() - a.y()));
            }
            remaining -= l;
        }
        return path.get(0);
    }

    private static double mod(double a, double m) {
        double r = a % m;
        return r < 0 ? r + m : r;
    }
}
