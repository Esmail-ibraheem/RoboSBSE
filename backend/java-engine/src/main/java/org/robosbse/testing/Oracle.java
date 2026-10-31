package org.robosbse.testing;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.Outcome;
import org.robosbse.simulation.RobotSpec;

/**
 * Test oracle: turns a simulated episode into property verdicts.
 *
 * <p>Distinguishing defects from impossible tasks: Phase 1 obstacles are static and every valid
 * scenario starts collision-free, so a robot can always avoid a collision by stopping (rotating in
 * place is collision-free for a disc). Hence any safety violation is attributable to the controller.
 * Liveness is different — a goal may be unreachable or the battery too small — so the liveness
 * verdict is only applicable when the planner confirms reachability and an energy estimate
 * confirms the battery suffices.
 */
public final class Oracle {

    /**
     * Facts about the scenario needed for liveness.
     *
     * @param reachable        goal reachable from start for the robot's footprint
     * @param energySufficient battery suffices for the shortest path at the configured speed
     * @param initialDistance  geodesic start-goal distance (m)
     * @param finalDistance    geodesic distance from the final position (m)
     */
    public record LivenessContext(boolean reachable, boolean energySufficient, double initialDistance,
                                  double finalDistance) {
    }

    private Oracle() {
    }

    public static List<Verdict> judge(EpisodeResult r, RobotSpec robot, double goalTolerance, double minClearance,
                                      Set<Property> properties, LivenessContext liveness) {
        List<Verdict> verdicts = new ArrayList<>();
        double radius = robot.radius();
        for (Property p : properties) {
            verdicts.add(switch (p) {
                case COLLISION -> {
                    boolean violated = r.outcome() == Outcome.COLLISION || r.minObstacleClearance() < 0;
                    yield new Verdict(p.id(), true, violated, clamp(r.minObstacleClearance() / radius),
                            String.format(Locale.ROOT, "min obstacle clearance %.3f m", r.minObstacleClearance()));
                }
                case BOUNDS -> {
                    boolean violated = r.outcome() == Outcome.OUT_OF_BOUNDS || r.minBorderClearance() < 0;
                    yield new Verdict(p.id(), true, violated, clamp(r.minBorderClearance() / radius),
                            String.format(Locale.ROOT, "min border clearance %.3f m", r.minBorderClearance()));
                }
                case VALID_COMMAND -> {
                    boolean violated = r.outcome() == Outcome.INVALID_COMMAND || r.outcome() == Outcome.CONTROLLER_ERROR;
                    yield new Verdict(p.id(), true, violated, violated ? -1 : 1,
                            violated ? (r.error() != null ? r.error() : "non-finite command") : "all commands finite");
                }
                case VELOCITY -> {
                    double excess = Math.max(r.maxCommandedSpeed() - robot.maxSpeed(),
                            r.maxCommandedTurnRate() - robot.maxTurnRate());
                    boolean violated = r.commandBoundViolations() > 0;
                    double rho = violated ? -0.01 - Math.min(0.99, Math.max(0, excess) / robot.maxSpeed()) : 1;
                    yield new Verdict(p.id(), true, violated, rho,
                            String.format(Locale.ROOT, "max commanded speed %.3f m/s (limit %.3f), %d violating steps",
                                    r.maxCommandedSpeed(), robot.maxSpeed(), r.commandBoundViolations()));
                }
                case COMPLETION -> {
                    if (!r.reportedDone()) {
                        yield new Verdict(p.id(), true, false, 1, "completion never reported");
                    }
                    boolean violated = r.outcome() == Outcome.FALSE_COMPLETION;
                    yield new Verdict(p.id(), true, violated,
                            clamp((goalTolerance - r.reportedDoneDistance()) / goalTolerance),
                            String.format(Locale.ROOT, "completion reported %.3f m from goal (tolerance %.3f)",
                                    r.reportedDoneDistance(), goalTolerance));
                }
                case CLEARANCE -> {
                    double margin = r.minObstacleClearance() - minClearance;
                    yield new Verdict(p.id(), true, margin < 0, clamp(margin / radius),
                            String.format(Locale.ROOT, "min clearance %.3f m (required %.3f)",
                                    r.minObstacleClearance(), minClearance));
                }
                case LIVENESS -> liveness(r, liveness);
            });
        }
        return verdicts;
    }

    private static Verdict liveness(EpisodeResult r, LivenessContext ctx) {
        String id = Property.LIVENESS.id();
        if (ctx == null || !ctx.reachable()) {
            return new Verdict(id, false, false, 1, "not applicable: goal unreachable for the robot footprint");
        }
        if (!ctx.energySufficient()) {
            return new Verdict(id, false, false, 1, "not applicable: battery insufficient for the shortest path");
        }
        if (r.success()) {
            return new Verdict(id, true, false, 1, "goal reached");
        }
        boolean safetyStop = switch (r.outcome()) {
            case COLLISION, OUT_OF_BOUNDS, INVALID_COMMAND, CONTROLLER_ERROR, FALSE_COMPLETION -> true;
            default -> false;
        };
        if (safetyStop) {
            return new Verdict(id, false, false, 1, "not judged: episode ended by a safety violation");
        }
        double progress = ctx.initialDistance() > 0 ? ctx.finalDistance() / ctx.initialDistance() : 1;
        return new Verdict(id, true, true, -Math.max(0.01, Math.min(1, progress)),
                String.format(Locale.ROOT, "%s with %.2f m left of %.2f m", r.outcome(), ctx.finalDistance(),
                        ctx.initialDistance()));
    }

    private static double clamp(double v) {
        return Math.max(-1, Math.min(1, v));
    }
}
