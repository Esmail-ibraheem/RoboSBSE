package org.robosbse.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Result of evaluating one candidate solution.
 *
 * <p>Convention used throughout RoboSBSE: <b>fitness is minimised</b>. Problems that naturally
 * maximise something (clearance, task completion) negate or invert it inside their fitness function
 * and document the transformation.
 *
 * @param fitness             scalar value that search algorithms minimise
 * @param constraintViolation non-negative degree of constraint violation; 0 means feasible
 * @param metrics             measured properties of the candidate (travel time, energy, ...)
 * @param objectives          objective vector for multi-objective algorithms (may be empty)
 * @param behaviour           behaviour descriptor for novelty search and behaviour coverage, with
 *                            components normalised to roughly [0, 1] (may be empty)
 */
public record Evaluation(double fitness, double constraintViolation, Map<String, Double> metrics,
                         double[] objectives, double[] behaviour) {

    public Evaluation {
        if (Double.isNaN(fitness)) {
            throw new IllegalArgumentException("fitness must not be NaN");
        }
        metrics = Collections.unmodifiableMap(new LinkedHashMap<>(metrics));
        objectives = objectives == null ? new double[0] : objectives.clone();
        behaviour = behaviour == null ? new double[0] : behaviour.clone();
    }

    public Evaluation(double fitness, double constraintViolation, Map<String, Double> metrics, double[] objectives) {
        this(fitness, constraintViolation, metrics, objectives, null);
    }

    public Evaluation(double fitness, double constraintViolation, Map<String, Double> metrics) {
        this(fitness, constraintViolation, metrics, null, null);
    }

    public static Evaluation of(double fitness) {
        return new Evaluation(fitness, 0.0, Map.of());
    }

    public boolean feasible() {
        return constraintViolation <= 0.0;
    }

    public double metric(String name, double fallback) {
        Double v = metrics.get(name);
        return v == null ? fallback : v;
    }

    @Override
    public double[] objectives() {
        return objectives.clone();
    }

    @Override
    public double[] behaviour() {
        return behaviour.clone();
    }

    /** Same evaluation with a behaviour descriptor attached. */
    public Evaluation withBehaviour(double[] descriptor) {
        return new Evaluation(fitness, constraintViolation, metrics, objectives, descriptor);
    }
}
