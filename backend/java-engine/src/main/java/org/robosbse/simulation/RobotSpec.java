package org.robosbse.simulation;

/**
 * Physical parameters of the differential-drive (unicycle) robot.
 *
 * @param radius                 body radius (m); the robot is a disc
 * @param maxSpeed               forward speed limit (m/s)
 * @param maxReverseSpeed        reverse speed limit (m/s, non-negative)
 * @param maxAcceleration        linear acceleration limit (m/s^2)
 * @param maxTurnRate            angular velocity limit (rad/s) — the turning constraint
 * @param maxAngularAcceleration angular acceleration limit (rad/s^2)
 * @param batteryCapacity        energy store (J)
 * @param idlePower              constant electronics draw (W)
 * @param speedPower             drive power per m/s (W s/m)
 * @param turnPower              power per rad/s of turning (W s/rad)
 * @param accelerationPower      power per m/s^2 of acceleration (W s^2/m)
 * @param sensorRays             number of range-sensor rays, evenly spread over 360 degrees
 * @param sensorRange            maximum sensing range (m)
 */
public record RobotSpec(double radius, double maxSpeed, double maxReverseSpeed, double maxAcceleration,
                        double maxTurnRate, double maxAngularAcceleration, double batteryCapacity,
                        double idlePower, double speedPower, double turnPower, double accelerationPower,
                        int sensorRays, double sensorRange) {

    public static final RobotSpec DEFAULT = new RobotSpec(0.25, 1.0, 0.0, 1.0, 2.0, 8.0, 1500.0,
            2.0, 10.0, 2.0, 4.0, 24, 4.0);

    /** Ray angle relative to the heading for ray {@code i} (0 = straight ahead). */
    public double rayAngle(int i) {
        return Math.PI * 2 * i / sensorRays;
    }

    /** Power draw (W) for the given velocities and acceleration. */
    public double power(double v, double omega, double acceleration) {
        return idlePower + speedPower * Math.abs(v) + turnPower * Math.abs(omega)
                + accelerationPower * Math.abs(acceleration);
    }
}
