package org.robosbse.algorithms.local;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Candidate;
import org.robosbse.core.Move;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Tabu search (Glover): best-admissible-move local search with short-term memory.
 *
 * <p>Every iteration evaluates the neighbourhood (or a random candidate list of it) and moves to
 * the best <em>admissible</em> neighbour — even if it is worse than the current solution, which is
 * how tabu search leaves local optima. After a move, the attribute of the move that would undo it
 * becomes tabu for {@code tenure} iterations. A tabu move is admissible only under the aspiration
 * criterion: it would produce a new best-so-far solution. If every move is tabu and none aspires,
 * the move whose tabu status expires first is taken. Optionally the search restarts from a random
 * solution after {@code maxStagnation} iterations without a new best.
 */
public final class TabuSearch implements SearchAlgorithm {

    /** Short-term memory: move attribute -> last iteration in which it is tabu. */
    public static final class TabuList {
        private final Map<Object, Long> tabuUntil = new HashMap<>();

        public void add(Object attribute, long untilIteration) {
            tabuUntil.put(attribute, untilIteration);
        }

        public boolean isTabu(Object attribute, long iteration) {
            Long until = tabuUntil.get(attribute);
            return until != null && iteration <= until;
        }

        long expiry(Object attribute) {
            return tabuUntil.getOrDefault(attribute, Long.MIN_VALUE);
        }

        public int activeCount(long iteration) {
            tabuUntil.values().removeIf(until -> until < iteration);
            return tabuUntil.size();
        }

        public void clear() {
            tabuUntil.clear();
        }
    }

    /** Outcome of move selection. */
    public record Choice(int index, boolean aspiration, boolean forced) {
    }

    private final Params params;
    private final int tenure;
    private final int candidates;
    private final boolean aspiration;
    private final int maxStagnation;

    public TabuSearch(Params params) {
        this.params = params;
        this.tenure = params.getInt("tenure", 7);
        this.candidates = params.getInt("candidates", 0);
        this.aspiration = params.getBool("aspiration", true);
        this.maxStagnation = params.getInt("maxStagnation", 0);
    }

    /**
     * Selects the move to make (exposed for testing): the best non-tabu move, where a tabu move is
     * allowed if aspiration is on and it beats {@code bestFitness}. If nothing is admissible, the
     * tabu move expiring soonest is forced.
     */
    public static Choice select(List<Double> fitness, List<Object> attributes, TabuList tabu, long iteration,
                                double bestFitness, boolean aspiration) {
        int chosen = -1;
        boolean aspired = false;
        for (int i = 0; i < fitness.size(); i++) {
            boolean isTabu = tabu.isTabu(attributes.get(i), iteration);
            boolean aspires = isTabu && aspiration && fitness.get(i) < bestFitness;
            if (isTabu && !aspires) {
                continue;
            }
            if (chosen < 0 || fitness.get(i) < fitness.get(chosen)) {
                chosen = i;
                aspired = aspires;
            }
        }
        if (chosen >= 0) {
            return new Choice(chosen, aspired, false);
        }
        int oldest = 0;
        for (int i = 1; i < attributes.size(); i++) {
            if (tabu.expiry(attributes.get(i)) < tabu.expiry(attributes.get(oldest))) {
                oldest = i;
            }
        }
        return new Choice(attributes.isEmpty() ? -1 : oldest, false, true);
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        Neighbourhood<S> neighbourhood = context.space().neighbourhood(params);
        TabuList tabu = new TabuList();
        Candidate<S> current = context.evaluate(context.space().random(context.rng()));
        long iteration = 0;
        long lastImprovement = 0;
        long aspirations = 0;
        long forced = 0;
        long restarts = 0;
        while (true) {
            List<Move<S>> moves = new ArrayList<>(neighbourhood.moves(current.solution()));
            if (moves.isEmpty()) {
                return;
            }
            if (candidates > 0 && moves.size() > candidates) {
                context.rng().shuffle(moves);
                moves = moves.subList(0, candidates);
            }
            List<Candidate<S>> evaluated = new ArrayList<>(moves.size());
            List<Double> fitness = new ArrayList<>(moves.size());
            List<Object> attributes = new ArrayList<>(moves.size());
            double bestBefore = context.best().fitness();
            for (Move<S> m : moves) {
                Candidate<S> c = context.evaluate(m.result());
                evaluated.add(c);
                fitness.add(c.fitness());
                attributes.add(m.attribute());
            }
            Choice choice = select(fitness, attributes, tabu, iteration, bestBefore, aspiration);
            if (choice.aspiration()) {
                aspirations++;
            }
            if (choice.forced()) {
                forced++;
            }
            current = evaluated.get(choice.index());
            tabu.add(moves.get(choice.index()).inverse(), iteration + tenure);
            iteration++;
            context.nextIteration();
            if (current.fitness() < bestBefore) {
                lastImprovement = iteration;
            }
            context.series("current", context.evaluations(), current.fitness());
            context.stat("tabuSize", tabu.activeCount(iteration));
            context.stat("aspirations", aspirations);
            context.stat("forcedMoves", forced);
            if (maxStagnation > 0 && iteration - lastImprovement >= maxStagnation) {
                current = context.evaluate(context.space().random(context.rng()));
                tabu.clear();
                lastImprovement = iteration;
                context.stat("restarts", ++restarts);
            }
        }
    }
}
