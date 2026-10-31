package org.robosbse.planning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;

class PlannerTest {

    private static World empty() {
        return new World(new WorldMap(null, "e", null, 10, 10, null, null, 0.3, List.of()));
    }

    @Test
    void octileOptimumOnAnEmptyGrid() {
        OccupancyGrid grid = new OccupancyGrid(empty(), 0.5, 0.0);
        // From cell (0,0) to cell (6,2): 2 diagonal + 4 straight moves of 0.5 m.
        GridPlanner.Plan plan = GridPlanner.astar(grid, new Vec2(0.25, 0.25), new Vec2(3.25, 1.25));
        assertTrue(plan.found());
        assertEquals(0.5 * (4 + 2 * Math.sqrt(2)), plan.gridLength(), 1e-9);
        assertEquals(new Vec2(3.25, 1.25).distanceTo(new Vec2(0.25, 0.25)), plan.smoothLength(), 1e-9,
                "line-of-sight shortcut is the straight segment");
    }

    @Test
    void astarAndDijkstraAgreeAndAstarExpandsFewerNodes() {
        for (String name : MapLibrary.BUILTIN) {
            WorldMap map = MapLibrary.builtin(name);
            OccupancyGrid grid = new OccupancyGrid(new World(map), 0.1, 0.35);
            GridPlanner.Plan a = GridPlanner.astar(grid, map.start().position(), map.goal());
            GridPlanner.Plan d = GridPlanner.dijkstra(grid, map.start().position(), map.goal());
            assertTrue(a.found(), name + ": built-in map must be solvable");
            assertEquals(d.gridLength(), a.gridLength(), 1e-9, name + ": both are optimal on the grid");
            assertTrue(a.expansions() <= d.expansions(), name);
            for (Vec2 cell : a.cells()) {
                assertFalse(grid.blocked(grid.cellOf(cell.x(), cell.y())), name + ": path through a blocked cell");
            }
        }
    }

    @Test
    void noPathThroughAClosedWall() {
        World walled = new World(new WorldMap(null, "w", null, 10, 10, null, null, 0.3,
                List.of(new Rect(4.8, -1, 0.4, 12))));
        OccupancyGrid grid = new OccupancyGrid(walled, 0.1, 0.3);
        GridPlanner.Plan plan = GridPlanner.astar(grid, new Vec2(1, 5), new Vec2(9, 5));
        assertFalse(plan.found());
        assertTrue(plan.expansions() > 0);
        assertFalse(new DistanceField(grid, new Vec2(9, 5)).reachable(1, 5));
    }

    @Test
    void noCornerCutting() {
        // Two blocks touching diagonally: the diagonal squeeze between them must not be used.
        World w = new World(new WorldMap(null, "c", null, 2, 2, null, null, 0.3,
                List.of(new Rect(0, 1, 1, 1), new Rect(1, 0, 1, 1))));
        OccupancyGrid grid = new OccupancyGrid(w, 1.0, 0.0);
        assertFalse(GridPlanner.astar(grid, new Vec2(0.5, 0.5), new Vec2(1.5, 1.5)).found());
    }

    @Test
    void geodesicDistanceIsAtLeastEuclidean() {
        WorldMap map = MapLibrary.builtin("maze");
        DistanceField field = new DistanceField(new OccupancyGrid(new World(map), 0.1, 0.25), map.goal());
        double geo = field.distanceFrom(map.start().x(), map.start().y());
        double euclid = map.start().position().distanceTo(map.goal());
        assertTrue(geo >= euclid - 0.2);
        assertTrue(geo > 1.5 * euclid, "the maze makes the geodesic distance much longer");
    }
}
