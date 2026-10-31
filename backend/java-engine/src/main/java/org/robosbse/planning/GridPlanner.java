package org.robosbse.planning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

import org.robosbse.geometry.Vec2;

/**
 * A* and Dijkstra on an 8-connected {@link OccupancyGrid} — the conventional reference planners.
 *
 * <p>Diagonal moves cost {@code sqrt(2)} cells and are only allowed when both adjacent orthogonal
 * cells are free (no corner cutting). A* uses the octile distance, which is admissible and
 * consistent on this graph, so both planners return a shortest grid path; A* simply expands fewer
 * nodes. Ties in the open list are broken towards larger g (deeper nodes), deterministically.
 */
public final class GridPlanner {

    private static final int[] DC = {1, -1, 0, 0, 1, 1, -1, -1};
    private static final int[] DR = {0, 0, 1, -1, 1, -1, 1, -1};
    private static final double SQRT2 = Math.sqrt(2);

    /**
     * @param found        whether a path exists
     * @param algorithm    {@code astar} or {@code dijkstra}
     * @param cells        grid path from start to goal (cell centres)
     * @param waypoints    line-of-sight shortcut of the grid path, ending at the exact goal
     * @param gridLength   length of the grid path (m)
     * @param smoothLength length of the shortcut path (m)
     * @param expansions   nodes taken from the open list
     * @param micros       planning time (microseconds)
     */
    public record Plan(boolean found, String algorithm, List<Vec2> cells, List<Vec2> waypoints, double gridLength,
                       double smoothLength, long expansions, long micros) {
    }

    private record Node(int index, double f, double g) {
    }

    private GridPlanner() {
    }

    public static Plan astar(OccupancyGrid grid, Vec2 start, Vec2 goal) {
        return plan(grid, start, goal, true);
    }

    public static Plan dijkstra(OccupancyGrid grid, Vec2 start, Vec2 goal) {
        return plan(grid, start, goal, false);
    }

    public static Plan plan(OccupancyGrid grid, Vec2 start, Vec2 goal, boolean heuristic) {
        long t0 = System.nanoTime();
        String name = heuristic ? "astar" : "dijkstra";
        int s = grid.cellOf(start.x(), start.y());
        int g = grid.cellOf(goal.x(), goal.y());
        if (grid.blocked(s) || grid.blocked(g)) {
            return new Plan(false, name, List.of(), List.of(), 0, 0, 0, (System.nanoTime() - t0) / 1000);
        }
        int n = grid.cols() * grid.rows();
        double[] cost = new double[n];
        Arrays.fill(cost, Double.POSITIVE_INFINITY);
        int[] parent = new int[n];
        Arrays.fill(parent, -1);
        boolean[] closed = new boolean[n];
        PriorityQueue<Node> open = new PriorityQueue<>((a, b) -> {
            int c = Double.compare(a.f, b.f);
            if (c != 0) {
                return c;
            }
            c = Double.compare(b.g, a.g);
            return c != 0 ? c : Integer.compare(a.index, b.index);
        });
        cost[s] = 0;
        open.add(new Node(s, heuristic ? octile(grid, s, g) : 0, 0));
        long expansions = 0;
        while (!open.isEmpty()) {
            Node node = open.poll();
            if (closed[node.index]) {
                continue;
            }
            closed[node.index] = true;
            expansions++;
            if (node.index == g) {
                break;
            }
            int c = grid.col(node.index);
            int r = grid.row(node.index);
            for (int k = 0; k < 8; k++) {
                int nc = c + DC[k];
                int nr = r + DR[k];
                if (grid.blocked(nc, nr)) {
                    continue;
                }
                if (k >= 4 && (grid.blocked(c + DC[k], r) || grid.blocked(c, r + DR[k]))) {
                    continue;
                }
                int ni = grid.index(nc, nr);
                if (closed[ni]) {
                    continue;
                }
                double step = (k >= 4 ? SQRT2 : 1.0) * grid.resolution() * grid.costFactor(ni);
                double ng = cost[node.index] + step;
                if (ng < cost[ni] - 1e-12) {
                    cost[ni] = ng;
                    parent[ni] = node.index;
                    open.add(new Node(ni, ng + (heuristic ? octile(grid, ni, g) : 0), ng));
                }
            }
        }
        if (!closed[g]) {
            return new Plan(false, name, List.of(), List.of(), 0, 0, expansions, (System.nanoTime() - t0) / 1000);
        }
        List<Vec2> cells = new ArrayList<>();
        for (int i = g; i != -1; i = parent[i]) {
            cells.add(grid.centre(i));
        }
        Collections.reverse(cells);
        List<Vec2> smooth = shortcut(grid, start, cells, goal);
        long micros = (System.nanoTime() - t0) / 1000;
        return new Plan(true, name, cells, smooth, cost[g], polylineLength(start, smooth), expansions, micros);
    }

    private static double octile(OccupancyGrid grid, int a, int b) {
        int dc = Math.abs(grid.col(a) - grid.col(b));
        int dr = Math.abs(grid.row(a) - grid.row(b));
        return (Math.max(dc, dr) + (SQRT2 - 1) * Math.min(dc, dr)) * grid.resolution();
    }

    /**
     * Greedy line-of-sight shortcutting: from the current point jump to the farthest later path
     * point that is directly visible in the inflated grid. Returns waypoints excluding the start.
     */
    static List<Vec2> shortcut(OccupancyGrid grid, Vec2 start, List<Vec2> cells, Vec2 goal) {
        List<Vec2> path = new ArrayList<>(cells);
        path.set(path.size() - 1, goal);
        List<Vec2> out = new ArrayList<>();
        Vec2 current = start;
        int i = 0;
        while (i < path.size()) {
            int farthest = i;
            for (int j = path.size() - 1; j > i; j--) {
                Vec2 p = path.get(j);
                if (grid.segmentFree(current.x(), current.y(), p.x(), p.y())) {
                    farthest = j;
                    break;
                }
            }
            current = path.get(farthest);
            out.add(current);
            i = farthest + 1;
        }
        return out;
    }

    public static double polylineLength(Vec2 start, List<Vec2> points) {
        double length = 0;
        Vec2 previous = start;
        for (Vec2 p : points) {
            length += previous.distanceTo(p);
            previous = p;
        }
        return length;
    }
}
