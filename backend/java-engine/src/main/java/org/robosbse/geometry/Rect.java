package org.robosbse.geometry;

import com.fasterxml.jackson.annotation.JsonIgnore;

/** Axis-aligned rectangle with lower-left corner (x, y), width w and height h. */
public record Rect(double x, double y, double w, double h) implements Shape {

    public Rect {
        if (!(w > 0) || !(h > 0)) {
            throw new IllegalArgumentException("rectangle size must be positive");
        }
    }

    @Override
    public double signedDistance(double px, double py) {
        double qx = Math.abs(px - (x + w / 2)) - w / 2;
        double qy = Math.abs(py - (y + h / 2)) - h / 2;
        double outside = Math.hypot(Math.max(qx, 0), Math.max(qy, 0));
        double inside = Math.min(Math.max(qx, qy), 0);
        return outside + inside;
    }

    @Override
    public double raycast(double ox, double oy, double dx, double dy) {
        if (ox >= x && ox <= x + w && oy >= y && oy <= y + h) {
            return 0;
        }
        double tMin = Double.NEGATIVE_INFINITY;
        double tMax = Double.POSITIVE_INFINITY;
        if (dx == 0) {
            if (ox < x || ox > x + w) {
                return Double.POSITIVE_INFINITY;
            }
        } else {
            double t1 = (x - ox) / dx;
            double t2 = (x + w - ox) / dx;
            tMin = Math.max(tMin, Math.min(t1, t2));
            tMax = Math.min(tMax, Math.max(t1, t2));
        }
        if (dy == 0) {
            if (oy < y || oy > y + h) {
                return Double.POSITIVE_INFINITY;
            }
        } else {
            double t1 = (y - oy) / dy;
            double t2 = (y + h - oy) / dy;
            tMin = Math.max(tMin, Math.min(t1, t2));
            tMax = Math.min(tMax, Math.max(t1, t2));
        }
        if (tMax < tMin || tMax < 0) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(tMin, 0);
    }

    @Override
    @JsonIgnore
    public Vec2 boundingCentre() {
        return new Vec2(x + w / 2, y + h / 2);
    }

    @Override
    @JsonIgnore
    public double boundingRadius() {
        return Math.hypot(w, h) / 2;
    }
}
