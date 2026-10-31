package org.robosbse.core;

import java.util.List;
import java.util.Map;

/**
 * Outcome of one search run. Every number in here was measured during the run; nothing is
 * estimated or synthesised afterwards.
 *
 * @param best                  best candidate found (null only if no evaluation happened)
 * @param evaluations           fitness evaluations consumed
 * @param elapsedMillis         active wall-clock time (pauses excluded)
 * @param stopReason            why the run ended
 * @param convergence           (evaluation, best fitness) at every improvement — an exact step curve
 * @param series                algorithm-specific series (diversity, temperature, ...)
 * @param stats                 final values of algorithm statistics
 * @param archive               best distinct candidates, ascending fitness
 * @param targets               first target hits (e.g. discovered failures), capped
 * @param targetCount           total number of evaluations that hit the target
 * @param firstTargetEvaluation evaluation index of the first target hit, or -1
 * @param iterations            algorithm iterations (generations, moves, ...)
 * @param pareto                non-dominated feasible candidates (multi-objective problems), else empty
 * @param hypervolume           hypervolume of {@code pareto} w.r.t. the problem's reference point, or NaN
 * @param cpuMillis             CPU time of the run: the running thread plus all evaluation threads
 * @param threads               evaluation threads the run used
 */
public record SearchResult<S>(Candidate<S> best, long evaluations, long elapsedMillis, StopReason stopReason,
                              List<double[]> convergence, Map<String, List<double[]>> series,
                              Map<String, Double> stats, List<Candidate<S>> archive,
                              List<Candidate<S>> targets, long targetCount, long firstTargetEvaluation,
                              long iterations, List<Candidate<S>> pareto, double hypervolume,
                              long cpuMillis, int threads) {

    /** Best-so-far fitness after {@code evaluation} evaluations, read off the step curve. */
    public double bestAt(long evaluation) {
        double value = Double.NaN;
        for (double[] point : convergence) {
            if (point[0] > evaluation) {
                break;
            }
            value = point[1];
        }
        return value;
    }
}
