package org.robosbse.arm;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;

/**
 * Forward and inverse kinematics and collision geometry of a planar serial arm.
 *
 * <ul>
 *   <li><b>Forward kinematics</b>: {@code p_0 = base}, {@code p_{i+1} = p_i + l_i (cos phi_i, sin phi_i)}
 *       with {@code phi_i = q_0 + ... + q_i}; the last point is the end effector.</li>
 *   <li><b>Analytical IK</b> (two links): {@code cos q_1 = (d^2 - l_0^2 - l_1^2) / (2 l_0 l_1)},
 *       {@code q_1 = +-acos(...)}, {@code q_0 = atan2(y, x) - atan2(l_1 sin q_1, l_0 + l_1 cos q_1)}: the
 *       two "elbow" solutions. Out of reach, the arm points straight at the target.</li>
 *   <li><b>Numerical IK</b> (any number of links): damped least squares (Levenberg-Marquardt),
 *       {@code dq = J^T (J J^T + lambda^2 I)^-1 e}, clamped to the joint limits.</li>
 *   <li><b>Collision</b>: every link is a segment of half-thickness {@code linkRadius}; its clearance to an
 *       obstacle is the minimum signed distance over points sampled every 2 cm along it, minus the
 *       half-thickness. Negative clearance means penetration.</li>
 * </ul>
 */
public final class ArmKinematics {

    private final ArmScene scene;
    private final double[] links;
    private final double[] min;
    private final double[] max;

    public ArmKinematics(ArmScene scene) {
        this.scene = scene;
        this.links = scene.links().stream().mapToDouble(Double::doubleValue).toArray();
        this.min = scene.jointMin().stream().mapToDouble(Double::doubleValue).toArray();
        this.max = scene.jointMax().stream().mapToDouble(Double::doubleValue).toArray();
    }

    public ArmScene scene() {
        return scene;
    }

    public int joints() {
        return links.length;
    }

    /** Base, joint and end-effector positions (n + 1 points). */
    public Vec2[] points(double[] q) {
        Vec2[] p = new Vec2[links.length + 1];
        double x = scene.base().x();
        double y = scene.base().y();
        double phi = 0;
        p[0] = new Vec2(x, y);
        for (int i = 0; i < links.length; i++) {
            phi += q[i];
            x += links[i] * Math.cos(phi);
            y += links[i] * Math.sin(phi);
            p[i + 1] = new Vec2(x, y);
        }
        return p;
    }

    public Vec2 endEffector(double[] q) {
        Vec2[] p = points(q);
        return p[p.length - 1];
    }

    /** Total amount by which the configuration exceeds the joint limits (rad); 0 when within. */
    public double limitViolation(double[] q) {
        double v = 0;
        for (int i = 0; i < q.length; i++) {
            v += Math.max(0, min[i] - q[i]) + Math.max(0, q[i] - max[i]);
        }
        return v;
    }

    /** Smallest clearance between any link and any obstacle (m); large when there are no obstacles. */
    public double clearance(double[] q) {
        List<Shape> obstacles = scene.obstacles();
        if (obstacles.isEmpty()) {
            return 10;
        }
        Vec2[] p = points(q);
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < links.length; i++) {
            Vec2 a = p[i];
            Vec2 b = p[i + 1];
            int samples = Math.max(2, (int) Math.ceil(links[i] / 0.02));
            for (int k = 0; k <= samples; k++) {
                double f = (double) k / samples;
                double x = a.x() + f * (b.x() - a.x());
                double y = a.y() + f * (b.y() - a.y());
                for (Shape s : obstacles) {
                    best = Math.min(best, s.signedDistance(x, y));
                }
            }
        }
        return best - scene.linkRadius();
    }

    /**
     * Analytical inverse kinematics of a two-link arm: both elbow solutions ({@code [0]} with positive
     * elbow angle). Throws for arms with more links.
     */
    public double[][] analyticalIk(Vec2 target) {
        if (links.length != 2) {
            throw new IllegalArgumentException("analytical IK is implemented for two-link arms");
        }
        double x = target.x() - scene.base().x();
        double y = target.y() - scene.base().y();
        double l0 = links[0];
        double l1 = links[1];
        double c = (x * x + y * y - l0 * l0 - l1 * l1) / (2 * l0 * l1);
        c = Math.max(-1, Math.min(1, c));
        double[][] out = new double[2][];
        int k = 0;
        for (double sign : new double[] {1, -1}) {
            double q1 = sign * Math.acos(c);
            double q0 = Math.atan2(y, x) - Math.atan2(l1 * Math.sin(q1), l0 + l1 * Math.cos(q1));
            out[k++] = new double[] {wrap(q0), q1};
        }
        return out;
    }

    /** Result of a numerical IK solve: configuration and the iterations it took. */
    public record Solve(double[] q, double error, int iterations) {
    }

    /** Damped-least-squares IK from {@code start}, clamped to the joint limits. */
    public Solve numericalIk(Vec2 target, double[] start, int maxIterations, double lambda) {
        double[] q = start.clone();
        int n = links.length;
        int it = 0;
        double error = Double.POSITIVE_INFINITY;
        for (; it < maxIterations; it++) {
            Vec2[] p = points(q);
            Vec2 e = p[n];
            double ex = target.x() - e.x();
            double ey = target.y() - e.y();
            error = Math.hypot(ex, ey);
            if (error < 1e-5) {
                break;
            }
            // Jacobian columns: d(end)/dq_i = (-(ye - y_i), xe - x_i).
            double[] jx = new double[n];
            double[] jy = new double[n];
            for (int i = 0; i < n; i++) {
                jx[i] = -(e.y() - p[i].y());
                jy[i] = e.x() - p[i].x();
            }
            double a = lambda * lambda;
            double d = lambda * lambda;
            double b = 0;
            for (int i = 0; i < n; i++) {
                a += jx[i] * jx[i];
                d += jy[i] * jy[i];
                b += jx[i] * jy[i];
            }
            double det = a * d - b * b;
            double ux = (d * ex - b * ey) / det;
            double uy = (-b * ex + a * ey) / det;
            for (int i = 0; i < n; i++) {
                q[i] = Math.max(min[i], Math.min(max[i], q[i] + jx[i] * ux + jy[i] * uy));
            }
        }
        return new Solve(q, error, it);
    }

    /** Joint-space straight line between two configurations, sampled so no joint moves more than {@code step}. */
    public static List<double[]> interpolate(double[] a, double[] b, double step) {
        double maxDelta = 0;
        for (int i = 0; i < a.length; i++) {
            maxDelta = Math.max(maxDelta, Math.abs(b[i] - a[i]));
        }
        int samples = Math.max(1, (int) Math.ceil(maxDelta / step));
        List<double[]> out = new ArrayList<>(samples + 1);
        for (int k = 0; k <= samples; k++) {
            double f = (double) k / samples;
            double[] q = new double[a.length];
            for (int i = 0; i < a.length; i++) {
                q[i] = a[i] + f * (b[i] - a[i]);
            }
            out.add(q);
        }
        return out;
    }

    static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
