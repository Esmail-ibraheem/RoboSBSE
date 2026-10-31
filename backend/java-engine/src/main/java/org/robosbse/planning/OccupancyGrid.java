package org.robosbse.planning;

import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.World;

/**
 * Uniform grid over the world in which a cell is blocked when its centre is closer than
 * {@code inflation} to an obstacle or the border (configuration-space obstacles for a disc robot).
 * The clearance of every cell centre is stored, so the grid doubles as a sampled distance field.
 *
 * <p>Static zones count too: a cell whose inflated disc overlaps a restricted zone is blocked, and each
 * cell carries a traversal cost factor {@code 1 / speedFactor} of the terrain at its centre (1 elsewhere).
 * Moving and timed obstacles are not part of the grid; planners therefore plan for the static world.
 */
public final class OccupancyGrid {

    private final double resolution;
    private final int cols;
    private final int rows;
    private final double inflation;
    private final double[] clearance;
    private final boolean[] blocked;
    private final double[] costFactor;

    public OccupancyGrid(World world, double resolution, double inflation) {
        if (!(resolution > 0)) {
            throw new IllegalArgumentException("resolution must be positive");
        }
        this.resolution = resolution;
        this.inflation = inflation;
        this.cols = Math.max(1, (int) Math.ceil(world.width() / resolution));
        this.rows = Math.max(1, (int) Math.ceil(world.height() / resolution));
        this.clearance = new double[cols * rows];
        this.blocked = new boolean[cols * rows];
        this.costFactor = new double[cols * rows];
        boolean zones = world.hasRestrictedZones();
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double d = world.distance(centreX(c), centreY(r));
                clearance[index(c, r)] = d;
                blocked[index(c, r)] = d < inflation || (zones && world.inRestrictedZone(centreX(c), centreY(r), inflation));
                costFactor[index(c, r)] = 1.0 / world.speedFactor(centreX(c), centreY(r));
            }
        }
    }

    /** Traversal cost multiplier of a cell (terrain: 1 / speed factor; 1 on normal ground). */
    public double costFactor(int index) {
        return costFactor[index];
    }

    public int cols() {
        return cols;
    }

    public int rows() {
        return rows;
    }

    public double resolution() {
        return resolution;
    }

    public double inflation() {
        return inflation;
    }

    public int index(int col, int row) {
        return row * cols + col;
    }

    public int col(int index) {
        return index % cols;
    }

    public int row(int index) {
        return index / cols;
    }

    public double centreX(int col) {
        return (col + 0.5) * resolution;
    }

    public double centreY(int row) {
        return (row + 0.5) * resolution;
    }

    public Vec2 centre(int index) {
        return new Vec2(centreX(col(index)), centreY(row(index)));
    }

    public boolean inside(int col, int row) {
        return col >= 0 && row >= 0 && col < cols && row < rows;
    }

    public boolean blocked(int col, int row) {
        return !inside(col, row) || blocked[index(col, row)];
    }

    public boolean blocked(int index) {
        return blocked[index];
    }

    public int cellOf(double x, double y) {
        int c = (int) Math.floor(x / resolution);
        int r = (int) Math.floor(y / resolution);
        c = Math.max(0, Math.min(cols - 1, c));
        r = Math.max(0, Math.min(rows - 1, r));
        return index(c, r);
    }

    /** Bilinearly interpolated clearance (signed distance to the nearest obstacle or border). */
    public double clearanceAt(double x, double y) {
        double gx = x / resolution - 0.5;
        double gy = y / resolution - 0.5;
        int c0 = (int) Math.floor(gx);
        int r0 = (int) Math.floor(gy);
        double tx = gx - c0;
        double ty = gy - r0;
        double v00 = sample(c0, r0);
        double v10 = sample(c0 + 1, r0);
        double v01 = sample(c0, r0 + 1);
        double v11 = sample(c0 + 1, r0 + 1);
        return (1 - ty) * ((1 - tx) * v00 + tx * v10) + ty * ((1 - tx) * v01 + tx * v11);
    }

    private double sample(int c, int r) {
        c = Math.max(0, Math.min(cols - 1, c));
        r = Math.max(0, Math.min(rows - 1, r));
        return clearance[index(c, r)];
    }

    /**
     * Whether the straight segment stays in free cells, sampled at a quarter of the resolution
     * (used for line-of-sight path shortcutting).
     */
    public boolean segmentFree(double ax, double ay, double bx, double by) {
        double length = Math.hypot(bx - ax, by - ay);
        int samples = Math.max(1, (int) Math.ceil(length / (resolution * 0.25)));
        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            double x = ax + t * (bx - ax);
            double y = ay + t * (by - ay);
            if (clearanceAt(x, y) < inflation) {
                return false;
            }
        }
        return true;
    }
}
