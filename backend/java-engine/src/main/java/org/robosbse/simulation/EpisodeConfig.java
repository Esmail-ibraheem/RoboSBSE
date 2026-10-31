package org.robosbse.simulation;

/**
 * Settings of one simulated episode.
 *
 * @param dt                fixed integration / control period (s)
 * @param maxSteps          step limit (episode time limit = dt * maxSteps)
 * @param autoComplete      end successfully as soon as the robot is within the goal tolerance; when
 *                          false only the controller's completion report ends the task
 * @param stopOnCollision   end the episode at the first collision (otherwise the robot is blocked)
 * @param deadlockWindow    seconds without progress that count as a deadlock (0 disables)
 * @param deadlockDistance  minimum displacement (m) within the window to count as progress
 * @param sensorNoise       standard deviation of additive Gaussian range noise (m)
 * @param scanDropout       probability that a whole range scan is lost in a step
 * @param initialBattery    initial charge as a fraction of the capacity
 * @param noiseSeed         seed of the sensor-noise random stream (common random numbers)
 * @param record            record a trajectory for replay
 * @param recordEvery       record every k-th step
 * @param recordRays        include range readings in the recorded frames
 */
public record EpisodeConfig(double dt, int maxSteps, boolean autoComplete, boolean stopOnCollision,
                            double deadlockWindow, double deadlockDistance, double sensorNoise, double scanDropout,
                            double initialBattery, long noiseSeed, boolean record, int recordEvery,
                            boolean recordRays) {

    public static EpisodeConfig defaults() {
        return new EpisodeConfig(0.05, 2400, true, true, 8.0, 0.15, 0.0, 0.0, 1.0, 1L, false, 1, false);
    }

    public EpisodeConfig withMaxSteps(int steps) {
        return new EpisodeConfig(dt, steps, autoComplete, stopOnCollision, deadlockWindow, deadlockDistance,
                sensorNoise, scanDropout, initialBattery, noiseSeed, record, recordEvery, recordRays);
    }

    public EpisodeConfig withAutoComplete(boolean value) {
        return new EpisodeConfig(dt, maxSteps, value, stopOnCollision, deadlockWindow, deadlockDistance,
                sensorNoise, scanDropout, initialBattery, noiseSeed, record, recordEvery, recordRays);
    }

    public EpisodeConfig withSensing(double noise, double dropout, long seed) {
        return new EpisodeConfig(dt, maxSteps, autoComplete, stopOnCollision, deadlockWindow, deadlockDistance,
                noise, dropout, initialBattery, seed, record, recordEvery, recordRays);
    }

    public EpisodeConfig withBattery(double fraction) {
        return new EpisodeConfig(dt, maxSteps, autoComplete, stopOnCollision, deadlockWindow, deadlockDistance,
                sensorNoise, scanDropout, fraction, noiseSeed, record, recordEvery, recordRays);
    }

    public EpisodeConfig recording(int every, boolean rays) {
        return new EpisodeConfig(dt, maxSteps, autoComplete, stopOnCollision, deadlockWindow, deadlockDistance,
                sensorNoise, scanDropout, initialBattery, noiseSeed, true, Math.max(1, every), rays);
    }
}
