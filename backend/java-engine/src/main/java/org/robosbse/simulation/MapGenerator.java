package org.robosbse.simulation;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.Rng;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.planning.DistanceField;
import org.robosbse.planning.OccupancyGrid;

/**
 * Seeded procedural map generation. Generated maps always keep the start and goal free and are
 * checked for reachability with the planner; obstacles that would disconnect start and goal are
 * removed again, so every generated navigation task is solvable.
 */
public final class MapGenerator {

    private MapGenerator() {
    }

    /** Random circles and boxes. */
    public static WorldMap scatter(long seed, double width, double height, int count, double robotRadius) {
        Rng rng = new Rng(seed);
        Pose start = new Pose(1.0, 1.0, 0);
        Vec2 goal = new Vec2(width - 1.0, height - 1.0);
        List<Shape> obstacles = new ArrayList<>();
        int attempts = 0;
        while (obstacles.size() < count && attempts < count * 50) {
            attempts++;
            Shape shape;
            if (rng.chance(0.5)) {
                double r = rng.uniform(0.3, 1.2);
                shape = new Circle(rng.uniform(r, width - r), rng.uniform(r, height - r), r);
            } else {
                double w = rng.uniform(0.4, 3.0);
                double h = rng.uniform(0.4, 3.0);
                shape = new Rect(rng.uniform(0, width - w), rng.uniform(0, height - h), w, h);
            }
            double keepOut = robotRadius + 0.8;
            if (shape.signedDistance(start.x(), start.y()) < keepOut || shape.signedDistance(goal.x(), goal.y()) < keepOut) {
                continue;
            }
            obstacles.add(shape);
            if (!solvable(width, height, start, goal, obstacles, robotRadius)) {
                obstacles.remove(obstacles.size() - 1);
            }
        }
        return new WorldMap(WorldMap.SCHEMA, "generated-scatter-" + seed,
                "Procedurally generated (seed " + seed + ", " + obstacles.size() + " obstacles).",
                width, height, start, goal, 0.3, obstacles);
    }

    /**
     * Perfect maze on a grid of {@code cellsX * cellsY} cells (randomised depth-first search), with
     * walls of the given thickness. Start is the lower-left cell, goal the upper-right cell.
     */
    public static WorldMap maze(long seed, int cellsX, int cellsY, double cellSize, double wall) {
        Rng rng = new Rng(seed);
        boolean[][] visited = new boolean[cellsX][cellsY];
        // east[x][y]: wall between (x,y) and (x+1,y); north[x][y]: wall between (x,y) and (x,y+1)
        boolean[][] east = new boolean[cellsX][cellsY];
        boolean[][] north = new boolean[cellsX][cellsY];
        for (int x = 0; x < cellsX; x++) {
            for (int y = 0; y < cellsY; y++) {
                east[x][y] = true;
                north[x][y] = true;
            }
        }
        List<int[]> stack = new ArrayList<>();
        stack.add(new int[] {0, 0});
        visited[0][0] = true;
        while (!stack.isEmpty()) {
            int[] cur = stack.get(stack.size() - 1);
            List<int[]> options = new ArrayList<>();
            int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (int[] d : dirs) {
                int nx = cur[0] + d[0];
                int ny = cur[1] + d[1];
                if (nx >= 0 && ny >= 0 && nx < cellsX && ny < cellsY && !visited[nx][ny]) {
                    options.add(new int[] {nx, ny, d[0], d[1]});
                }
            }
            if (options.isEmpty()) {
                stack.remove(stack.size() - 1);
                continue;
            }
            int[] next = rng.pick(options);
            if (next[2] == 1) {
                east[cur[0]][cur[1]] = false;
            } else if (next[2] == -1) {
                east[next[0]][next[1]] = false;
            } else if (next[3] == 1) {
                north[cur[0]][cur[1]] = false;
            } else {
                north[next[0]][next[1]] = false;
            }
            visited[next[0]][next[1]] = true;
            stack.add(new int[] {next[0], next[1]});
        }
        List<Shape> walls = new ArrayList<>();
        double half = wall / 2;
        for (int x = 0; x < cellsX; x++) {
            for (int y = 0; y < cellsY; y++) {
                if (east[x][y] && x < cellsX - 1) {
                    walls.add(new Rect((x + 1) * cellSize - half, y * cellSize - half, wall, cellSize + wall));
                }
                if (north[x][y] && y < cellsY - 1) {
                    walls.add(new Rect(x * cellSize - half, (y + 1) * cellSize - half, cellSize + wall, wall));
                }
            }
        }
        double width = cellsX * cellSize;
        double height = cellsY * cellSize;
        return new WorldMap(WorldMap.SCHEMA, "generated-maze-" + seed,
                "Procedurally generated perfect maze (seed " + seed + ").", width, height,
                new Pose(cellSize / 2, cellSize / 2, 0), new Vec2(width - cellSize / 2, height - cellSize / 2), 0.3,
                walls);
    }

    private static boolean solvable(double width, double height, Pose start, Vec2 goal, List<Shape> obstacles,
                                    double robotRadius) {
        World world = new World(new WorldMap(null, "probe", null, width, height, start, goal, 0.3, obstacles));
        OccupancyGrid grid = new OccupancyGrid(world, 0.2, robotRadius + 0.15);
        return new DistanceField(grid, goal).reachable(start.x(), start.y());
    }
}
