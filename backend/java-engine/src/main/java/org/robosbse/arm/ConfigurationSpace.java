package org.robosbse.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Configuration-space motion planner for two-link arms: a grid over the joint ranges (resolution
 * {@code res} rad) whose cells are free when the arm in that configuration keeps a clearance of at least
 * {@code margin}, and A* over the 8-connected grid with the cost {@code max(|dq_0|, |dq_1|)}: the time of
 * a synchronised move at full joint speed, so the planned path is time-optimal on the grid. The cell path
 * is then shortcut greedily wherever the straight joint-space segment stays collision-free (checked at
 * the motion model's resolution), giving a few keyframes.
 *
 * <p>This is the conventional motion-planning reference for the arm problems; its node expansions are
 * reported separately from fitness evaluations.
 */
public final class ConfigurationSpace {

    private final ArmKinematics kin;
    private final double res;
    private final double margin;
    private final double lo0;
    private final double lo1;
    private final int n0;
    private final int n1;
    private final boolean[] free;
    private long expansions;

    public ConfigurationSpace(ArmKinematics kin, double res, double margin) {
        if (kin.joints() != 2) {
            throw new IllegalArgumentException("the configuration-space planner is implemented for two-link arms");
        }
        this.kin = kin;
        this.res = res;
        this.margin = margin;
        this.lo0 = kin.scene().jointMin().get(0);
        this.lo1 = kin.scene().jointMin().get(1);
        this.n0 = (int) Math.floor((kin.scene().jointMax().get(0) - lo0) / res) + 1;
        this.n1 = (int) Math.floor((kin.scene().jointMax().get(1) - lo1) / res) + 1;
        this.free = new boolean[n0 * n1];
        for (int i = 0; i < n0; i++) {
            for (int j = 0; j < n1; j++) {
                free[i * n1 + j] = kin.clearance(new double[] {lo0 + i * res, lo1 + j * res}) >= margin;
            }
        }
    }

    public long expansions() {
        return expansions;
    }

    /** Fraction of the joint ranges that is collision-free. */
    public double freeFraction() {
        int f = 0;
        for (boolean b : free) {
            f += b ? 1 : 0;
        }
        return (double) f / free.length;
    }

    private int cell(double[] q) {
        int i = (int) Math.round((q[0] - lo0) / res);
        int j = (int) Math.round((q[1] - lo1) / res);
        if (i < 0 || j < 0 || i >= n0 || j >= n1) {
            return -1;
        }
        return i * n1 + j;
    }

    private double[] config(int c) {
        return new double[] {lo0 + (c / n1) * res, lo1 + (c % n1) * res};
    }

    /** Planned motion: keyframes from {@code a} to {@code b}, or null if they are not connected. */
    public List<double[]> plan(double[] a, double[] b) {
        int s = cell(a);
        int g = cell(b);
        if (s < 0 || g < 0 || !free[s] || !free[g]) {
            return null;
        }
        double[] cost = new double[free.length];
        int[] parent = new int[free.length];
        Arrays.fill(cost, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        boolean[] closed = new boolean[free.length];
        PriorityQueue<double[]> open = new PriorityQueue<>((x, y) -> Double.compare(x[0], y[0]));
        cost[s] = 0;
        open.add(new double[] {heuristic(s, g), s});
        while (!open.isEmpty()) {
            int c = (int) open.poll()[1];
            if (closed[c]) {
                continue;
            }
            closed[c] = true;
            expansions++;
            if (c == g) {
                break;
            }
            int ci = c / n1;
            int cj = c % n1;
            for (int di = -1; di <= 1; di++) {
                for (int dj = -1; dj <= 1; dj++) {
                    if (di == 0 && dj == 0) {
                        continue;
                    }
                    int ni = ci + di;
                    int nj = cj + dj;
                    if (ni < 0 || nj < 0 || ni >= n0 || nj >= n1) {
                        continue;
                    }
                    int nc = ni * n1 + nj;
                    if (!free[nc] || closed[nc]) {
                        continue;
                    }
                    double nCost = cost[c] + res;
                    if (nCost < cost[nc] - 1e-12) {
                        cost[nc] = nCost;
                        parent[nc] = c;
                        open.add(new double[] {nCost + heuristic(nc, g), nc});
                    }
                }
            }
        }
        if (parent[g] < 0 && g != s) {
            return null;
        }
        List<double[]> cells = new ArrayList<>();
        for (int c = g; c >= 0; c = parent[c]) {
            cells.add(0, config(c));
            if (c == s) {
                break;
            }
        }
        cells.set(0, a.clone());
        cells.set(cells.size() - 1, b.clone());
        return shortcut(cells);
    }

    private double heuristic(int c, int g) {
        return Math.max(Math.abs(c / n1 - g / n1), Math.abs(c % n1 - g % n1)) * res;
    }

    /** Greedy shortcutting: from each kept keyframe jump to the farthest one reachable in a straight line. */
    private List<double[]> shortcut(List<double[]> path) {
        List<double[]> out = new ArrayList<>();
        int i = 0;
        out.add(path.get(0));
        while (i < path.size() - 1) {
            int j = path.size() - 1;
            while (j > i + 1 && !straightFree(path.get(i), path.get(j))) {
                j--;
            }
            out.add(path.get(j));
            i = j;
        }
        return out;
    }

    boolean straightFree(double[] a, double[] b) {
        for (double[] q : ArmKinematics.interpolate(a, b, ArmMotion.CHECK_STEP)) {
            if (kin.clearance(q) < margin) {
                return false;
            }
        }
        return true;
    }
}
