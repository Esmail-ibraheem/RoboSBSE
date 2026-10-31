package org.robosbse.algorithms.local;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.robosbse.core.Candidate;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;

/**
 * Simulated annealing with the Metropolis acceptance rule.
 *
 * <p>A random neighbour with fitness change {@code delta} is always accepted when it is not worse
 * and otherwise accepted with probability {@code exp(-delta / T)}. The temperature {@code T}
 * follows a configurable {@link CoolingSchedule} and is updated every {@code stepsPerTemperature}
 * moves (the Markov-chain length).
 *
 * <p>If {@code initialTemperature <= 0} it is calibrated automatically: a few random neighbours of
 * the start solution are evaluated (charged to the budget) and {@code T0} is set so that the median
 * worsening move would be accepted with probability {@code initialAcceptance}. If the geometric
 * factor {@code alpha <= 0} it is derived from the budget so that the temperature decays to
 * {@code finalTemperatureRatio * T0} exactly when the budget is used up.
 */
public final class SimulatedAnnealing implements SearchAlgorithm {

    private final Params params;
    private final double configuredT0;
    private final String coolingName;
    private final double alpha;
    private final double beta;
    private final double finalTemperatureRatio;
    private final double minTemperature;
    private final int stepsPerTemperature;
    private final double initialAcceptance;
    private final int calibrationSamples;

    public SimulatedAnnealing(Params params) {
        this.params = params;
        this.configuredT0 = params.getDouble("initialTemperature", 0);
        this.coolingName = params.getString("cooling", "geometric");
        this.alpha = params.getDouble("alpha", 0);
        this.beta = params.getDouble("beta", 0.01);
        this.finalTemperatureRatio = params.getDouble("finalTemperatureRatio", 1e-3);
        CoolingSchedule.byName(coolingName, 0.99, beta); // validates the name eagerly
        this.minTemperature = params.getDouble("minTemperature", 1e-9);
        this.stepsPerTemperature = params.getInt("stepsPerTemperature", 1);
        this.initialAcceptance = params.getDouble("initialAcceptance", 0.8);
        this.calibrationSamples = params.getInt("calibrationSamples", 20);
    }

    /**
     * The Metropolis acceptance rule, exposed for testing.
     *
     * @param delta       fitness change (candidate minus current; negative is an improvement)
     * @param temperature current temperature
     * @param uniform     a uniform random number in [0, 1)
     */
    public static boolean accept(double delta, double temperature, double uniform) {
        if (delta <= 0) {
            return true;
        }
        if (temperature <= 0) {
            return false;
        }
        return uniform < Math.exp(-delta / temperature);
    }

    @Override
    public <S> void run(SearchContext<S> context) {
        Neighbourhood<S> neighbourhood = context.space().neighbourhood(params);
        Candidate<S> current = context.evaluate(context.space().random(context.rng()));
        double t0 = configuredT0 > 0 ? configuredT0 : calibrate(context, neighbourhood, current);
        long totalSteps = Math.max(1, context.remainingEvaluations() / stepsPerTemperature);
        double effectiveAlpha = alpha > 0 ? alpha : Math.pow(finalTemperatureRatio, 1.0 / totalSteps);
        CoolingSchedule cooling = CoolingSchedule.byName(coolingName, effectiveAlpha, beta);
        context.stat("alpha", effectiveAlpha);
        double temperature = t0;
        long step = 0;
        long moves = 0;
        long accepted = 0;
        long worseAccepted = 0;
        context.stat("initialTemperature", t0);
        while (true) {
            Candidate<S> candidate = context.evaluate(neighbourhood.random(current.solution(), context.rng()));
            double delta = candidate.fitness() - current.fitness();
            if (accept(delta, temperature, context.rng().nextDouble())) {
                if (delta > 0) {
                    worseAccepted++;
                }
                current = candidate;
                accepted++;
            }
            moves++;
            context.nextIteration();
            if (moves % stepsPerTemperature == 0) {
                step++;
                temperature = Math.max(minTemperature, cooling.next(temperature, step, t0, totalSteps));
            }
            if (moves % 25 == 0) {
                context.series("temperature", context.evaluations(), temperature);
                context.series("current", context.evaluations(), current.fitness());
            }
            context.stat("temperature", temperature);
            context.stat("acceptanceRate", (double) accepted / moves);
            context.stat("worseAccepted", worseAccepted);
        }
    }

    private <S> double calibrate(SearchContext<S> context, Neighbourhood<S> neighbourhood, Candidate<S> start) {
        List<Double> worsening = new ArrayList<>();
        int samples = (int) Math.min(calibrationSamples, Math.max(1, context.remainingEvaluations() / 20));
        for (int i = 0; i < samples; i++) {
            Candidate<S> c = context.evaluate(neighbourhood.random(start.solution(), context.rng()));
            double delta = c.fitness() - start.fitness();
            if (delta > 0) {
                worsening.add(delta);
            }
        }
        if (worsening.isEmpty()) {
            return 1.0;
        }
        Collections.sort(worsening);
        double median = worsening.get(worsening.size() / 2);
        return -median / Math.log(initialAcceptance);
    }

    public String coolingName() {
        return coolingName;
    }
}
