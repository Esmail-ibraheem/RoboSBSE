package org.robosbse.planning;

import java.util.Arrays;
import java.util.PriorityQueue;

import org.robosbse.geometry.Vec2;

/**
 * Geodesic (obstacle-aware) distance to a goal for every free grid cell, computed once with
 * Dijkstra from the goal. It serves as a navigation function: the progress term of the navigation
 * fitness, and the reachability check that separates impossible test scenarios from real failures.
 */
public final class DistanceField {

    private final OccupancyGrid grid;
    private final double[] distance;
    private final Vec2 goal;

    public DistanceField(OccupancyGrid grid, Vec2 goal) {
        this.grid = grid;
        this.goal = goal;
        int n = grid.cols() * grid.rows();
        distance = new double[n];
        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        int g = grid.cellOf(goal.x(), goal.y());
        if (grid.blocked(g)) {
            return;
        }
        PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        distance[g] = 0;
        open.add(new double[] {0, g});
        int[] dc = {1, -1, 0, 0, 1, 1, -1, -1};
        int[] dr = {0, 0, 1, -1, 1, -1, 1, -1};
        while (!open.isEmpty()) {
            double[] top = open.poll();
            int idx = (int) top[1];
            if (top[0] > distance[idx]) {
                continue;
            }
            int c = grid.col(idx);
            int r = grid.row(idx);
            for (int k = 0; k < 8; k++) {
                int nc = c + dc[k];
                int nr = r + dr[k];
                if (grid.blocked(nc, nr)) {
                    continue;
                }
                if (k >= 4 && (grid.blocked(c + dc[k], r) || grid.blocked(c, r + dr[k]))) {
                    continue;
                }
                int ni = grid.index(nc, nr);
                double nd = distance[idx] + (k >= 4 ? Math.sqrt(2) : 1.0) * grid.resolution();
                if (nd < distance[ni]) {
                    distance[ni] = nd;
                    open.add(new double[] {nd, ni});
                }
            }
        }
    }

    public OccupancyGrid grid() {
        return grid;
    }

    /** Whether the goal is reachable from the given position through free cells. */
    public boolean reachable(double x, double y) {
        return Double.isFinite(cellDistance(x, y));
    }

    /**
     * Geodesic distance from (x, y) to the goal. Points in blocked or unreachable cells use the
     * nearest reachable cell within a small radius plus the straight distance to it; if none exists
     * the Euclidean distance is returned with a penalty factor of 2, so the value always decreases
     * towards the goal and stays finite.
     */
    public double distanceFrom(double x, double y) {
        double d = cellDistance(x, y);
        if (Double.isFinite(d)) {
            return d + Math.hypot(x - grid.centreX(grid.col(grid.cellOf(x, y))), y - grid.centreY(grid.row(grid.cellOf(x, y))));
        }
        int centre = grid.cellOf(x, y);
        int c0 = grid.col(centre);
        int r0 = grid.row(centre);
        double best = Double.POSITIVE_INFINITY;
        int radius = (int) Math.ceil(1.0 / grid.resolution());
        for (int dr = -radius; dr <= radius; dr++) {
            for (int dc = -radius; dc <= radius; dc++) {
                int c = c0 + dc;
                int r = r0 + dr;
                if (!grid.inside(c, r)) {
                    continue;
                }
                double cd = distance[grid.index(c, r)];
                if (Double.isFinite(cd)) {
                    best = Math.min(best, cd + Math.hypot(x - grid.centreX(c), y - grid.centreY(r)));
                }
            }
        }
        return Double.isFinite(best) ? best : 2 * Math.hypot(x - goal.x(), y - goal.y());
    }

    private double cellDistance(double x, double y) {
        return distance[grid.cellOf(x, y)];
    }
}
