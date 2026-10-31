package org.robosbse.algorithms.local;

import java.util.List;

/**
 * Temperature schedule for simulated annealing. {@code step} counts temperature updates, starting
 * at 0 for the initial temperature; {@code totalSteps} is the number of updates the budget allows
 * (used by the linear schedule).
 */
public interface CoolingSchedule {

    List<String> NAMES = List.of("geometric", "linear", "logarithmic", "lundy-mees");

    double next(double current, long step, double initial, long totalSteps);

    /** Geometric (exponential) cooling: T(k+1) = alpha * T(k). */
    static CoolingSchedule geometric(double alpha) {
        return (current, step, initial, total) -> current * alpha;
    }

    /** Linear cooling from T0 to 0 over the available steps: T(k) = T0 * (1 - k / K). */
    static CoolingSchedule linear() {
        return (current, step, initial, total) -> initial * Math.max(0.0, 1.0 - (double) step / Math.max(1, total));
    }

    /** Logarithmic cooling (Geman and Geman): T(k) = T0 / ln(e + k). Very slow but classical. */
    static CoolingSchedule logarithmic() {
        return (current, step, initial, total) -> initial / Math.log(Math.E + step);
    }

    /** Lundy and Mees: T(k+1) = T(k) / (1 + beta * T(k)). */
    static CoolingSchedule lundyMees(double beta) {
        return (current, step, initial, total) -> current / (1.0 + beta * current);
    }

    static CoolingSchedule byName(String name, double alpha, double beta) {
        return switch (name) {
            case "geometric" -> geometric(alpha);
            case "linear" -> linear();
            case "logarithmic" -> logarithmic();
            case "lundy-mees" -> lundyMees(beta);
            default -> throw new IllegalArgumentException("unknown cooling schedule '" + name + "', expected " + NAMES);
        };
    }
}
