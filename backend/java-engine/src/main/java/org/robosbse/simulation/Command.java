package org.robosbse.simulation;

/**
 * Controller output for one time step.
 *
 * @param v     requested linear velocity (m/s)
 * @param omega requested angular velocity (rad/s)
 * @param done  the controller declares its task complete (checked by the completion oracle)
 */
public record Command(double v, double omega, boolean done) {

    public static Command drive(double v, double omega) {
        return new Command(v, omega, false);
    }

    public static Command stop() {
        return new Command(0, 0, false);
    }

    public static Command complete() {
        return new Command(0, 0, true);
    }

    public boolean finite() {
        return Double.isFinite(v) && Double.isFinite(omega);
    }
}
