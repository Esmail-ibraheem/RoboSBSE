package org.robosbse.testing;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

import org.robosbse.core.Params;
import org.robosbse.core.representation.RealVector;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.controllers.ControllerRegistry;

/** Creating, reproducing and minimising failure records. */
public final class Failures {

    private Failures() {
    }

    /** Builds a failure record for a scenario, including a differential run of the correct controller. */
    public static FailureRecord record(ControllerTestingProblem problem, Scenario scenario, RealVector genome,
                                       Map<String, Object> provenance, Map<String, Object> minimization) {
        ControllerTestingProblem.Execution ex = problem.execute(scenario, problem.controllerId(), false);
        Map<String, Object> reference = null;
        String correct = ControllerRegistry.get(problem.controllerId()).faultyOf();
        if (correct != null) {
            ControllerTestingProblem.Execution ref = problem.execute(scenario, correct, false);
            reference = new LinkedHashMap<>();
            reference.put("controller", correct);
            reference.put("violated", ref.violated());
            reference.put("outcome", ref.result().outcome().name());
        }
        return new FailureRecord(FailureRecord.SCHEMA, problem.controllerId(),
                new LinkedHashMap<>(problem.controllerParams().asMap()), ex.violated(), ex.verdicts(),
                ex.result().outcome().name(), ex.result().time(), scenario, problem.arena(),
                new LinkedHashMap<>(problem.params().asMap()), problem.robot(),
                genome == null ? null : genome.toArray(), reference, provenance, minimization);
    }

    /** Rebuilds the testing problem described by a record. */
    public static ControllerTestingProblem problemOf(FailureRecord record) {
        Map<String, Object> testing = new LinkedHashMap<>(record.testing());
        testing.put("controller", record.controller());
        return new ControllerTestingProblem(record.arena(), Params.of(testing),
                Params.of(record.controllerParams()), record.robot());
    }

    /**
     * Outcome of replaying a record.
     *
     * @param reproduced         every recorded violation occurred again
     * @param violated           properties violated in the replay
     * @param referenceViolated  properties the correct controller violates on the same scenario
     *                           (null if the controller is not a known faulty variant)
     */
    public record Reproduction(boolean reproduced, List<String> violated, List<String> referenceViolated,
                               ControllerTestingProblem.Execution execution) {
    }

    public static Reproduction reproduce(FailureRecord record, boolean recordTrajectory) {
        ControllerTestingProblem problem = problemOf(record);
        ControllerTestingProblem.Execution ex = problem.execute(record.scenario(), record.controller(), recordTrajectory);
        List<String> violated = ex.violated();
        boolean reproduced = !record.violated().isEmpty() && violated.containsAll(record.violated());
        List<String> referenceViolated = null;
        String correct = ControllerRegistry.get(record.controller()).faultyOf();
        if (correct != null) {
            referenceViolated = problem.execute(record.scenario(), correct, false).violated();
        }
        return new Reproduction(reproduced, violated, referenceViolated, ex);
    }

    /**
     * Result of failure reduction.
     *
     * @param scenario    reduced scenario that still violates {@code property}
     * @param simulations episodes simulated during reduction
     * @param steps       description of every accepted simplification
     */
    public record Minimized(Scenario scenario, String property, int simulations, List<String> steps) {
    }

    /**
     * Failure minimisation. Every candidate simplification is kept only if the scenario stays valid
     * (including the start/goal keep-out of obstacles) and the controller still violates the same
     * property:
     * <ol>
     *   <li><b>ddmin</b> (Zeller and Hildebrandt) over the obstacle set: tries subsets and complements
     *       at increasing granularity, giving a 1-minimal set (removing any single remaining obstacle
     *       makes the failure disappear);</li>
     *   <li><b>numeric shrinking</b> of scan dropout and sensor noise towards 0 and of the initial
     *       battery towards full: first the neutral value itself, otherwise {@value #BISECTION_STEPS}
     *       bisection steps to the still-failing value closest to it;</li>
     *   <li><b>rounding</b> of all positions, sizes and angles to 0.1, else 0.01 (metres / radians),
     *       which makes the scenario easy to read and to re-type.</li>
     * </ol>
     * The steps list records every accepted change.
     */
    public static Minimized minimize(ControllerTestingProblem problem, Scenario scenario, String property) {
        String controller = problem.controllerId();
        int[] simulations = {0};
        Predicate<Scenario> fails = s -> {
            simulations[0]++;
            return stillFails(problem, s, controller, property);
        };
        List<String> steps = new ArrayList<>();
        Scenario current = scenario;

        int before = current.obstacles().size();
        Scenario base = current;
        List<Shape> kept = ddmin(current.obstacles(), subset -> fails.test(base.withObstacles(subset)));
        if (kept.size() < before) {
            current = current.withObstacles(kept);
            steps.add("ddmin: " + before + " -> " + kept.size() + " obstacles");
        }
        current = shrink(current, "scan dropout", Scenario::scanDropout,
                (s, v) -> s.withSensing(s.sensorNoise(), v), 0.0, fails, steps);
        current = shrink(current, "sensor noise", Scenario::sensorNoise,
                (s, v) -> s.withSensing(v, s.scanDropout()), 0.0, fails, steps);
        current = shrink(current, "battery", Scenario::battery, Scenario::withBattery, 1.0, fails, steps);
        for (double step : new double[] {0.1, 0.01}) {
            Scenario rounded = round(current, step);
            if (!rounded.equals(current) && fails.test(rounded)) {
                current = rounded;
                steps.add("rounded positions, sizes and angles to " + step);
                break;
            }
        }
        return new Minimized(current, property, simulations[0], steps);
    }

    static final int BISECTION_STEPS = 10;

    /** Delta debugging over a list; see {@link org.robosbse.core.DeltaDebugging#ddmin}. */
    static <T> List<T> ddmin(List<T> items, Predicate<List<T>> test) {
        return org.robosbse.core.DeltaDebugging.ddmin(items, test);
    }

    /** Moves one numeric input towards its neutral value while the failure persists. */
    private static Scenario shrink(Scenario s, String name, ToDoubleFunction<Scenario> get,
                                   BiFunction<Scenario, Double, Scenario> set, double neutral,
                                   Predicate<Scenario> fails, List<String> steps) {
        double original = get.applyAsDouble(s);
        if (original == neutral) {
            return s;
        }
        if (fails.test(set.apply(s, neutral))) {
            steps.add(name + " set to " + neutral);
            return set.apply(s, neutral);
        }
        double passing = neutral;
        double failing = original;
        for (int i = 0; i < BISECTION_STEPS; i++) {
            double mid = (passing + failing) / 2;
            if (fails.test(set.apply(s, mid))) {
                failing = mid;
            } else {
                passing = mid;
            }
        }
        if (failing == original) {
            return s;
        }
        steps.add(String.format(Locale.ROOT, "%s moved from %.4f to %.4f (bisection towards %.1f)", name, original,
                failing, neutral));
        return set.apply(s, failing);
    }

    /** All positions, sizes and angles rounded to multiples of {@code step}. */
    static Scenario round(Scenario s, double step) {
        List<Shape> obstacles = new ArrayList<>();
        for (Shape shape : s.obstacles()) {
            obstacles.add(switch (shape) {
                case Circle c -> new Circle(r(c.x(), step), r(c.y(), step), Math.max(step, r(c.r(), step)));
                case Rect q -> new Rect(r(q.x(), step), r(q.y(), step), Math.max(step, r(q.w(), step)),
                        Math.max(step, r(q.h(), step)));
            });
        }
        return new Scenario(new Pose(r(s.start().x(), step), r(s.start().y(), step), r(s.start().theta(), step)),
                new Vec2(r(s.goal().x(), step), r(s.goal().y(), step)), s.sensorNoise(), s.scanDropout(), s.battery(),
                s.speedLimit(), s.freeDistance(), obstacles);
    }

    private static double r(double value, double step) {
        return Math.round(value / step) * step;
    }

    private static boolean stillFails(ControllerTestingProblem problem, Scenario s, String controller, String property) {
        return problem.invalidity(s) == 0 && problem.obstaclesClearOfEndpoints(s)
                && problem.execute(s, controller, false).violated().contains(property);
    }
}
