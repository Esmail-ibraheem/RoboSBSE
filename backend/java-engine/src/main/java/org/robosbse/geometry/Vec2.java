package org.robosbse.geometry;

/** Immutable 2D point / vector (x to the right, y up; angles counter-clockwise from +x). */
public record Vec2(double x, double y) {

    public static final Vec2 ZERO = new Vec2(0, 0);

    public Vec2 plus(Vec2 o) {
        return new Vec2(x + o.x, y + o.y);
    }

    public Vec2 minus(Vec2 o) {
        return new Vec2(x - o.x, y - o.y);
    }

    public Vec2 times(double s) {
        return new Vec2(x * s, y * s);
    }

    public double dot(Vec2 o) {
        return x * o.x + y * o.y;
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double distanceTo(Vec2 o) {
        return Math.hypot(x - o.x, y - o.y);
    }

    public double distanceTo(double px, double py) {
        return Math.hypot(x - px, y - py);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public static Vec2 polar(double length, double angle) {
        return new Vec2(length * Math.cos(angle), length * Math.sin(angle));
    }
}
