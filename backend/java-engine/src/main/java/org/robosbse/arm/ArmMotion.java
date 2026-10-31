package org.robosbse.arm;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.geometry.Vec2;

/**
 * Motion model for a sequence of keyframe configurations, joined by straight lines in joint space.
 *
 * <p>Each segment is a synchronised move: all joints start and stop together, and the segment lasts
 * {@code max_i |dq_i| / maxJointSpeed}. Measured over the whole motion:
 * <ul>
 *   <li>{@code time}: sum of segment durations plus optional pauses at keyframes (s);</li>
 *   <li>{@code travel}: total joint travel {@code sum |dq_i|} (rad), the movement cost;</li>
 *   <li>{@code effort}: {@code sum_seg sum_i I_i dq_i^2 / T_seg} with {@code I_i = (l_i + ... + l_n)^2}, i.e.
 *       the integral of the inertia-weighted squared joint speed, a simple energy model for a
 *       horizontal arm (no gravity, no friction);</li>
 *   <li>{@code roughness}: {@code sum_k |dq_{k+1} - dq_k|} over interior keyframes (direction changes in
 *       joint space; 0 for a straight move), the smoothness term;</li>
 *   <li>{@code penetration}: integral of {@code max(0, -clearance)} over the path, sampled so that no joint
 *       moves more than 0.02 rad between checks, and the minimum clearance;</li>
 *   <li>{@code limits}: joint-limit violation summed over the keyframes (straight segments between
 *       keyframes inside the limits stay inside, the limits being a box).</li>
 * </ul>
 */
public final class ArmMotion {

    static final double CHECK_STEP = 0.02;

    private ArmMotion() {
    }

    /** Measurements of one motion. */
    public record Stats(double time, double travel, double effort, double roughness, double penetration,
                        double minClearance, double limits, int collidingSamples) {

        public Map<String, Double> metrics() {
            Map<String, Double> m = new LinkedHashMap<>();
            m.put("time", time);
            m.put("travel", travel);
            m.put("effort", effort);
            m.put("roughness", roughness);
            m.put("penetration", penetration);
            m.put("minClearance", Math.min(minClearance, 10));
            m.put("limitViolation", limits);
            m.put("collidingSamples", (double) collidingSamples);
            return m;
        }

        public boolean collisionFree() {
            return collidingSamples == 0;
        }
    }

    public static Stats measure(ArmKinematics kin, List<double[]> keyframes, double[] pauses) {
        ArmScene scene = kin.scene();
        int n = kin.joints();
        double[] inertia = new double[n];
        double outer = 0;
        for (int i = n - 1; i >= 0; i--) {
            outer += scene.links().get(i);
            inertia[i] = outer * outer;
        }
        double time = 0;
        double travel = 0;
        double effort = 0;
        double roughness = 0;
        double penetration = 0;
        double minClearance = Double.POSITIVE_INFINITY;
        double limits = 0;
        int colliding = 0;
        double[] previousDelta = null;
        for (int k = 0; k < keyframes.size(); k++) {
            limits += kin.limitViolation(keyframes.get(k));
            if (pauses != null && k < pauses.length) {
                time += pauses[k];
            }
        }
        if (keyframes.size() == 1) {
            double c = kin.clearance(keyframes.get(0));
            return new Stats(time, 0, 0, 0, 0, c, limits, c < 0 ? 1 : 0);
        }
        for (int k = 1; k < keyframes.size(); k++) {
            double[] a = keyframes.get(k - 1);
            double[] b = keyframes.get(k);
            double[] delta = new double[n];
            double maxDelta = 0;
            for (int i = 0; i < n; i++) {
                delta[i] = b[i] - a[i];
                travel += Math.abs(delta[i]);
                maxDelta = Math.max(maxDelta, Math.abs(delta[i]));
            }
            double duration = maxDelta / scene.maxJointSpeed();
            time += duration;
            if (duration > 0) {
                for (int i = 0; i < n; i++) {
                    effort += inertia[i] * delta[i] * delta[i] / duration;
                }
            }
            if (previousDelta != null) {
                double s = 0;
                for (int i = 0; i < n; i++) {
                    double d = delta[i] - previousDelta[i];
                    s += d * d;
                }
                roughness += Math.sqrt(s);
            }
            previousDelta = delta;
            List<double[]> samples = ArmKinematics.interpolate(a, b, CHECK_STEP);
            double sampleSpan = maxDelta / Math.max(1, samples.size() - 1);
            for (int s = k == 1 ? 0 : 1; s < samples.size(); s++) {
                double c = kin.clearance(samples.get(s));
                minClearance = Math.min(minClearance, c);
                if (c < 0) {
                    colliding++;
                    penetration += -c * Math.max(sampleSpan, 1e-3);
                }
            }
        }
        return new Stats(time, travel, effort, roughness, penetration, minClearance, limits, colliding);
    }

    /** Replay frames every {@code dt} seconds: time, joint positions, clearance and keyframe index. */
    public static List<Map<String, Object>> frames(ArmKinematics kin, List<double[]> keyframes, double[] pauses, double dt) {
        List<Map<String, Object>> out = new ArrayList<>();
        double t = 0;
        double maxSpeed = kin.scene().maxJointSpeed();
        for (int k = 0; k < keyframes.size(); k++) {
            double pause = pauses != null && k < pauses.length ? pauses[k] : 0;
            if (k > 0) {
                double[] a = keyframes.get(k - 1);
                double[] b = keyframes.get(k);
                double maxDelta = 0;
                for (int i = 0; i < a.length; i++) {
                    maxDelta = Math.max(maxDelta, Math.abs(b[i] - a[i]));
                }
                double duration = maxDelta / maxSpeed;
                int steps = Math.max(1, (int) Math.ceil(duration / dt));
                for (int s = 1; s <= steps; s++) {
                    double f = (double) s / steps;
                    double[] q = new double[a.length];
                    for (int i = 0; i < a.length; i++) {
                        q[i] = a[i] + f * (b[i] - a[i]);
                    }
                    out.add(frame(kin, t + f * duration, q, k - 1));
                }
                t += duration;
            } else {
                out.add(frame(kin, 0, keyframes.get(0), 0));
            }
            if (pause > 0) {
                t += pause;
                out.add(frame(kin, t, keyframes.get(k), k));
            }
        }
        return out;
    }

    private static Map<String, Object> frame(ArmKinematics kin, double t, double[] q, int segment) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("t", t);
        List<double[]> points = new ArrayList<>();
        for (Vec2 p : kin.points(q)) {
            points.add(new double[] {p.x(), p.y()});
        }
        m.put("points", points);
        m.put("q", q.clone());
        m.put("clearance", Math.min(10, kin.clearance(q)));
        m.put("segment", segment);
        return m;
    }
}
