package org.robosbse.geometry;

import com.fasterxml.jackson.annotation.JsonIgnore;

/** Circle with centre (x, y) and radius r. */
public record Circle(double x, double y, double r) implements Shape {

    public Circle {
        if (!(r > 0)) {
            throw new IllegalArgumentException("circle radius must be positive");
        }
    }

    @Override
    public double signedDistance(double px, double py) {
        return Math.hypot(px - x, py - y) - r;
    }

    @Override
    public double raycast(double ox, double oy, double dx, double dy) {
        double fx = ox - x;
        double fy = oy - y;
        double c = fx * fx + fy * fy - r * r;
        if (c <= 0) {
            return 0;
        }
        double b = fx * dx + fy * dy;
        double disc = b * b - c;
        if (disc < 0) {
            return Double.POSITIVE_INFINITY;
        }
        double t = -b - Math.sqrt(disc);
        return t >= 0 ? t : Double.POSITIVE_INFINITY;
    }

    @Override
    @JsonIgnore
    public Vec2 boundingCentre() {
        return new Vec2(x, y);
    }

    @Override
    @JsonIgnore
    public double boundingRadius() {
        return r;
    }
}
