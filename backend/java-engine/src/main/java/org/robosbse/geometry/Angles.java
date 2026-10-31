package org.robosbse.geometry;

/** Angle helpers. */
public final class Angles {

    private Angles() {
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrap(double angle) {
        double a = angle % (2 * Math.PI);
        if (a <= -Math.PI) {
            a += 2 * Math.PI;
        } else if (a > Math.PI) {
            a -= 2 * Math.PI;
        }
        return a;
    }

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
