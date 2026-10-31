package org.robosbse.simulation;

/**
 * What a controller perceives at one time step.
 *
 * <p>The pose is exact (localisation is out of scope for Phase 1). Range readings come from the
 * simulated distance sensor: {@code ranges[i]} is measured along heading-relative angle
 * {@link RobotSpec#rayAngle(int)}; a reading equal to {@code sensorRange} means "nothing detected".
 * When the whole scan is lost ({@code scanValid == false}) every entry is {@code NaN}.
 */
public record Observation(double time, double x, double y, double theta, double v, double omega,
                          double battery, boolean scanValid, double[] ranges) {

    public double range(int i) {
        return ranges[i];
    }

    public int rays() {
        return ranges == null ? 0 : ranges.length;
    }
}
