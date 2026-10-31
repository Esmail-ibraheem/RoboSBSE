package org.robosbse.simulation;

import org.robosbse.geometry.Vec2;

/** Position and heading (radians, counter-clockwise from +x). */
public record Pose(double x, double y, double theta) {

    public Vec2 position() {
        return new Vec2(x, y);
    }
}
