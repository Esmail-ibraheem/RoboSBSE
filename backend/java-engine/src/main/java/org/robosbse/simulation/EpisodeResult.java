package org.robosbse.simulation;

import java.util.LinkedHashMap;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonInclude;

/**
 * Measured outcome of one episode. All quantities come from the time-stepped simulation.
 *
 * @param outcome                 termination reason
 * @param steps                   simulated steps
 * @param time                    simulated time (s)
 * @param distance                distance travelled (m)
 * @param energy                  energy consumed (J)
 * @param collisions              number of collision events
 * @param minObstacleClearance    minimum distance between robot body and any obstacle (m; negative
 *                                means penetration)
 * @param minBorderClearance      minimum distance between robot body and the world border (m)
 * @param smoothness              total variation of the angular velocity, sum |omega_k - omega_k-1|
 *                                (rad/s; lower is smoother)
 * @param finalPose               pose at the end of the episode
 * @param finalDistanceToGoal     distance from the robot centre to the goal at the end (m)
 * @param maxCommandedSpeed       largest commanded linear velocity (m/s)
 * @param maxCommandedTurnRate    largest commanded |angular velocity| (rad/s)
 * @param commandBoundViolations  steps whose command exceeded the robot's velocity limits
 * @param reportedDone            whether the controller reported completion
 * @param reportedDoneDistance    distance to goal when completion was reported (NaN if never)
 * @param droppedScans            number of lost sensor scans
 * @param error                   controller exception message, if any
 * @param restrictedTime          time the robot body overlapped a restricted zone (s)
 * @param dynamicCollision        whether a collision involved a moving or timed obstacle (such a
 *                                collision may be unavoidable for the robot)
 * @param pathSamples             robot position at 25%, 50% and 75% of the episode's duration,
 *                                as {@code [x25, y25, x50, y50, x75, y75]}
 * @param trajectory              recorded trajectory or null
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public record EpisodeResult(Outcome outcome, int steps, double time, double distance, double energy,
                            int collisions, double minObstacleClearance, double minBorderClearance,
                            double smoothness, Pose finalPose, double finalDistanceToGoal,
                            double maxCommandedSpeed, double maxCommandedTurnRate, int commandBoundViolations,
                            boolean reportedDone, double reportedDoneDistance, int droppedScans, String error,
                            double restrictedTime, boolean dynamicCollision, double[] pathSamples, Trajectory trajectory) {

    public EpisodeResult {
        pathSamples = pathSamples == null ? new double[6] : pathSamples.clone();
    }

    @Override
    public double[] pathSamples() {
        return pathSamples.clone();
    }

    /**
     * Behaviour descriptor of the episode: final position followed by the positions at 75%, 50% and
     * 25% of the episode, all divided by the world size so that every component lies in [0, 1].
     * Used by novelty search and by behaviour coverage.
     */
    public double[] behaviour(double worldWidth, double worldHeight) {
        return new double[] {
                finalPose.x() / worldWidth, finalPose.y() / worldHeight,
                pathSamples[4] / worldWidth, pathSamples[5] / worldHeight,
                pathSamples[2] / worldWidth, pathSamples[3] / worldHeight,
                pathSamples[0] / worldWidth, pathSamples[1] / worldHeight};
    }

    public boolean success() {
        return outcome.success();
    }

    /** Numeric metrics for reports and fitness bookkeeping. */
    public Map<String, Double> metrics() {
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("success", success() ? 1.0 : 0.0);
        m.put("time", time);
        m.put("distance", distance);
        m.put("energy", energy);
        m.put("collisions", (double) collisions);
        m.put("minClearance", minObstacleClearance);
        m.put("minBorderClearance", minBorderClearance);
        m.put("smoothness", smoothness);
        m.put("finalDistance", finalDistanceToGoal);
        m.put("maxCommandedSpeed", maxCommandedSpeed);
        m.put("commandBoundViolations", (double) commandBoundViolations);
        m.put("droppedScans", (double) droppedScans);
        if (restrictedTime > 0) {
            m.put("restrictedTime", restrictedTime);
        }
        if (dynamicCollision) {
            m.put("dynamicCollision", 1.0);
        }
        return m;
    }
}
