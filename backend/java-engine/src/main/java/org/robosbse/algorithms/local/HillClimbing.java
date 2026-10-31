package org.robosbse.algorithms.local;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Hill climbing (local search with a strict-improvement acceptance rule) in three variants:
 *
 * <ul>
 *   <li><b>first</b> improvement: scans the enumerable neighbourhood in random order and moves to the
 *       first strictly better neighbour;</li>
 *   <li><b>best</b> improvement (steepest ascent): evaluates the whole neighbourhood and moves to
 *       the best neighbour if it is strictly better;</li>
 *   <li><b>stochastic</b>: samples random neighbours and moves on improvement; after
 *       {@code maxFailures} consecutive non-improving samples the point is treated as a local
 *       optimum (with high probability).</li>
 * </ul>
 *
 * When a local optimum is reached the climber restarts from a new random solution if
 * {@code restarts} is enabled (random-restart hill climbing); otherwise the run finishes early.
 */
public final class HillClimbing implements SearchAlgorithm {

    private final Params params;
    private final String variant;
    private final boolean restarts;
    private final int maxFailures;
    private final boolean acceptEqual;

    public HillClimbing(Params params) {
        this.params = params;
        this.variant = params.getString("variant", "first");
        this.restarts = params.getBool("restarts", true);
        this.maxFailures = params.getInt("maxFailures", 50);
        this.acceptEqual = params.getBool("acceptEqual", false);
        if (!List.of("first", "best", "stochastic").contains(variant)) {
            throw new IllegalArgumentException("unknown hill-climbing variant: " + variant);
        }
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        Neighbourhood<S> neighbourhood = context.space().neighbourhood(params);
        int climbs = 0;
        while (true) {
            Candidate<S> current = context.evaluate(context.space().random(context.rng()));
            climbs++;
            context.stat("climbs", climbs);
            current = climb(context, neighbourhood, current);
            context.series("localOptimum", context.evaluations(), current.fitness());
            if (!restarts) {
                return;
            }
        }
    }

    /** Climbs from {@code start} until no improving neighbour is found; returns the local optimum. */
    private <S> Candidate<S> climb(SearchContext<S> context, Neighbourhood<S> neighbourhood, Candidate<S> start) {
        Candidate<S> current = start;
        while (true) {
            Candidate<S> next = switch (variant) {
                case "first" -> firstImprovement(context, neighbourhood, current);
                case "best" -> bestImprovement(context, neighbourhood, current);
                default -> stochastic(context, neighbourhood, current);
            };
            if (next == null) {
                return current;
            }
            current = next;
            context.nextIteration();
            context.series("current", context.evaluations(), current.fitness());
        }
    }

    private boolean improves(Candidate<?> candidate, Candidate<?> current) {
        return candidate.fitness() < current.fitness()
                || (acceptEqual && candidate.fitness() == current.fitness());
    }

    private <S> Candidate<S> firstImprovement(SearchContext<S> context, Neighbourhood<S> neighbourhood,
                                              Candidate<S> current) {
        List<S> neighbours = new ArrayList<>(neighbourhood.all(current.solution()));
        context.rng().shuffle(neighbours);
        for (S neighbour : neighbours) {
            Candidate<S> candidate = context.evaluate(neighbour);
            if (candidate.fitness() < current.fitness()) {
                return candidate;
            }
        }
        return null;
    }

    private <S> Candidate<S> bestImprovement(SearchContext<S> context, Neighbourhood<S> neighbourhood,
                                             Candidate<S> current) {
        Candidate<S> best = null;
        for (S neighbour : neighbourhood.all(current.solution())) {
            Candidate<S> candidate = context.evaluate(neighbour);
            if (best == null || candidate.fitness() < best.fitness()) {
                best = candidate;
            }
        }
        return best != null && best.fitness() < current.fitness() ? best : null;
    }

    private <S> Candidate<S> stochastic(SearchContext<S> context, Neighbourhood<S> neighbourhood,
                                        Candidate<S> current) {
        for (int failures = 0; failures < maxFailures; failures++) {
            Candidate<S> candidate = context.evaluate(neighbourhood.random(current.solution(), context.rng()));
            if (improves(candidate, current)) {
                return candidate;
            }
        }
        return null;
    }
}
