package org.robosbse.algorithms;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.robosbse.algorithms.baseline.ExhaustiveSearch;
import org.robosbse.algorithms.baseline.RandomSearch;
import org.robosbse.algorithms.baseline.RandomWalk;
import org.robosbse.algorithms.evolutionary.EvolutionStrategy;
import org.robosbse.algorithms.evolutionary.GeneticAlgorithm;
import org.robosbse.algorithms.evolutionary.ParticleSwarm;
import org.robosbse.algorithms.evolutionary.Selection;
import org.robosbse.algorithms.local.CoolingSchedule;
import org.robosbse.algorithms.local.HillClimbing;
import org.robosbse.algorithms.local.SimulatedAnnealing;
import org.robosbse.algorithms.local.TabuSearch;
import org.robosbse.algorithms.multiobjective.Nsga2;
import org.robosbse.algorithms.memetic.MemeticAlgorithm;
import org.robosbse.algorithms.novelty.NoveltySearch;
import org.robosbse.algorithms.parallel.IslandGeneticAlgorithm;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.representation.PermutationSpace;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geneticprogramming.ProgramSpace;
import org.robosbse.patching.PatchSpace;

/**
 * Catalogue of available search algorithms with their parameter declarations. Adding an algorithm
 * means implementing {@link SearchAlgorithm} and registering a descriptor here; the UI picks it up
 * automatically from the engine's capability message.
 */
public final class AlgorithmRegistry {

    /**
     * Static description of an algorithm.
     *
     * @param id              identifier used in configuration files
     * @param name            display name
     * @param family          grouping (baseline, local, evolutionary, swarm, multi-objective)
     * @param summary         one-paragraph description shown in the UI
     * @param params          declared parameters (only these are accepted)
     * @param requires        operator kinds the representation must provide
     * @param representations representations it supports ({@code any}, {@code real-vector}, {@code permutation})
     * @param multiObjective  whether it needs a problem with an objective vector
     */
    public record Descriptor(String id, String name, String family, String summary, List<ParamSpec> params,
                             List<String> requires, List<String> representations, boolean multiObjective,
                             Function<Params, SearchAlgorithm> factory) {

        public SearchAlgorithm create(Params params) {
            List<String> errors = ParamSpec.validate(this.params, params);
            if (!errors.isEmpty()) {
                throw new IllegalArgumentException("invalid parameters for " + id + ": " + String.join("; ", errors));
            }
            return factory.apply(withDefaults(params));
        }

        /** Fills in declared defaults for absent parameters. */
        public Params withDefaults(Params params) {
            Map<String, Object> merged = new LinkedHashMap<>();
            for (ParamSpec spec : this.params) {
                merged.put(spec.name(), spec.defaultValue());
            }
            merged.putAll(params.asMap());
            return Params.of(merged);
        }

        /** Whether the algorithm can run on a problem with this representation and objectives. */
        public boolean supports(String representation, boolean hasObjectives) {
            return (representations.contains("any") || representations.contains(representation))
                    && (!multiObjective || hasObjectives);
        }
    }

    private static final List<String> ANY = List.of("any");
    private static final List<String> REAL = List.of("real-vector");

    private static final ParamSpec STEP_SIZE = ParamSpec.real("stepSize", 0.05, 1e-4, 1.0,
            "Real vectors: neighbourhood step as a fraction of each variable's range (sigma of the random neighbour).");
    private static final ParamSpec NEIGHBOURHOOD = ParamSpec.choice("neighbourhood",
            union("default", List.of(), PermutationSpace.NEIGHBOURHOODS),
            "Permutations: swap, insert or inversion moves (real vectors always use the gene-step neighbourhood).");
    private static final ParamSpec MUTATION = ParamSpec.choice("mutation",
            union("default", RealVectorSpace.MUTATIONS, PermutationSpace.MUTATIONS, ProgramSpace.MUTATIONS, PatchSpace.MUTATIONS),
            "Mutation operator; 'default' = the representation's first operator.");
    private static final ParamSpec CROSSOVER = ParamSpec.choice("crossover",
            union("default", RealVectorSpace.CROSSOVERS, PermutationSpace.CROSSOVERS, ProgramSpace.CROSSOVERS, PatchSpace.CROSSOVERS),
            "Crossover operator; 'default' = the representation's first operator.");
    private static final ParamSpec MUTATION_RATE = ParamSpec.real("mutationRate", 0, 0, 1,
            "Real vectors: per-gene mutation probability; 0 = 1/n. Permutations apply one move per mutation.");
    private static final ParamSpec MUTATION_SIGMA = ParamSpec.real("mutationSigma", 0.1, 1e-6, 1,
            "Real vectors: Gaussian mutation sigma as a fraction of the range.");

    private static final Map<String, Descriptor> DESCRIPTORS = new LinkedHashMap<>();

    static {
        register(new Descriptor("random-search", "Random Search", "baseline",
                "Samples independent uniformly random solutions and keeps the best. Uses no feedback; the "
                        + "reference baseline every metaheuristic must beat.",
                List.of(), List.of("random"), ANY, false, p -> new RandomSearch()));

        register(new Descriptor("random-walk", "Random Walk", "baseline",
                "Moves to a random neighbour at every step regardless of fitness. Isolates the effect of the "
                        + "acceptance rule used by local search.",
                List.of(STEP_SIZE, NEIGHBOURHOOD), List.of("neighbourhood"), ANY, false, RandomWalk::new));

        register(new Descriptor("exhaustive-search", "Exhaustive Search", "baseline",
                "Evaluates every solution of a small enumerable space (permutations). If it completes within the "
                        + "budget, its best solution is a proven optimum.",
                List.of(), List.of("enumerate"), List.of("permutation"), false, p -> new ExhaustiveSearch()));

        register(new Descriptor("hill-climbing", "Hill Climbing", "local",
                "Moves to strictly better neighbours until none exists (local optimum), then optionally restarts "
                        + "from a random solution. Variants: first improvement, best improvement (steepest "
                        + "ascent) and stochastic.",
                List.of(
                        ParamSpec.choice("variant", List.of("first", "best", "stochastic"),
                                "first = first improving neighbour in random order; best = evaluate the whole "
                                        + "neighbourhood; stochastic = sample random neighbours."),
                        STEP_SIZE,
                        NEIGHBOURHOOD,
                        ParamSpec.bool("restarts", true, "Restart from a random solution at local optima."),
                        ParamSpec.integer("maxFailures", 50, 1, 100000,
                                "Stochastic variant: consecutive non-improving samples before declaring a local optimum."),
                        ParamSpec.bool("acceptEqual", false,
                                "Stochastic variant: also accept equally good neighbours (plateau moves).")),
                List.of("neighbourhood"), ANY, false, HillClimbing::new));

        register(new Descriptor("simulated-annealing", "Simulated Annealing", "local",
                "Accepts worse neighbours with probability exp(-delta/T) (Metropolis rule) while the temperature "
                        + "T decreases according to a cooling schedule, allowing escapes from local optima.",
                List.of(
                        ParamSpec.real("initialTemperature", 0, 0, 1e9,
                                "Initial temperature; 0 = calibrate from sampled worsening moves."),
                        ParamSpec.real("initialAcceptance", 0.8, 0.01, 0.99,
                                "Calibration target: probability of accepting the median worsening move at T0."),
                        ParamSpec.choice("cooling", CoolingSchedule.NAMES, "Cooling schedule."),
                        ParamSpec.real("alpha", 0, 0, 0.999999,
                                "Geometric cooling factor per temperature step; 0 = derive from the budget."),
                        ParamSpec.real("finalTemperatureRatio", 1e-3, 1e-12, 1,
                                "With alpha = 0: T at the end of the budget as a fraction of T0."),
                        ParamSpec.real("beta", 0.01, 1e-9, 10, "Lundy-Mees cooling parameter."),
                        ParamSpec.real("minTemperature", 1e-9, 0, 1e6, "Temperature floor."),
                        ParamSpec.integer("stepsPerTemperature", 1, 1, 100000,
                                "Moves per temperature step (Markov chain length)."),
                        ParamSpec.integer("calibrationSamples", 20, 1, 1000,
                                "Neighbours sampled for automatic T0 calibration (charged to the budget)."),
                        STEP_SIZE,
                        NEIGHBOURHOOD),
                List.of("neighbourhood"), ANY, false, SimulatedAnnealing::new));

        register(new Descriptor("tabu-search", "Tabu Search", "local",
                "Moves to the best admissible neighbour even if it is worse; the move undoing it stays tabu for "
                        + "'tenure' iterations unless it would give a new best (aspiration).",
                List.of(
                        ParamSpec.integer("tenure", 7, 1, 100000, "Iterations a reversing move stays tabu."),
                        ParamSpec.integer("candidates", 0, 0, 1000000,
                                "Random candidate-list size per iteration; 0 = the whole neighbourhood."),
                        ParamSpec.bool("aspiration", true, "Allow tabu moves that produce a new best-so-far solution."),
                        ParamSpec.integer("maxStagnation", 0, 0, 1000000,
                                "Restart from a random solution after this many iterations without a new best; 0 = never."),
                        STEP_SIZE,
                        NEIGHBOURHOOD),
                List.of("neighbourhood-moves"), ANY, false, TabuSearch::new));

        List<ParamSpec> gaParams = List.of(
                ParamSpec.integer("populationSize", 50, 2, 10000, "Number of individuals."),
                ParamSpec.choice("selection", Selection.NAMES, "Parent selection scheme."),
                ParamSpec.integer("tournamentSize", 3, 1, 100, "Tournament size (tournament selection)."),
                ParamSpec.real("rankPressure", 1.5, 1.0, 2.0, "Selection pressure (rank selection)."),
                CROSSOVER,
                ParamSpec.real("crossoverRate", 0.8, 0, 1, "Probability of recombining a parent pair."),
                MUTATION,
                MUTATION_RATE,
                MUTATION_SIGMA);
        List<ParamSpec> generational = new ArrayList<>(gaParams);
        generational.add(ParamSpec.integer("elitism", 2, 0, 1000, "Best parents copied unchanged (generational replacement)."));
        generational.add(ParamSpec.choice("replacement", List.of("generational", "plus", "steady-state"),
                "generational = offspring replace parents except elites; plus = (mu+lambda) truncation; "
                        + "steady-state = each child replaces the worst if better."));
        register(new Descriptor("genetic-algorithm", "Genetic Algorithm", "evolutionary",
                "Evolves a population with selection, crossover and mutation; elitism keeps the best "
                        + "individuals. Records best/mean/worst fitness and diversity per generation.",
                generational, List.of("random", "mutation", "crossover"), ANY, false, GeneticAlgorithm::new));

        register(new Descriptor("steady-state-ga", "Steady-State GA", "evolutionary",
                "Genetic algorithm that inserts offspring one at a time: each child replaces the current worst "
                        + "individual if it is better, so the population best never gets worse.",
                gaParams, List.of("random", "mutation", "crossover"), ANY, false,
                p -> new GeneticAlgorithm(p.with("replacement", "steady-state").with("elitism", 0))));

        register(new Descriptor("evolution-strategy", "Evolution Strategy", "evolutionary",
                "(mu/rho +, lambda)-ES with intermediate recombination, Gaussian mutation and self-adaptive or "
                        + "1/5-success-rule step sizes. Real vectors only.",
                List.of(
                        ParamSpec.integer("mu", 5, 1, 10000, "Number of parents."),
                        ParamSpec.integer("lambda", 35, 1, 100000, "Offspring per generation."),
                        ParamSpec.integer("rho", 2, 1, 10000, "Parents recombined per offspring (1 = no recombination)."),
                        ParamSpec.choice("selection", List.of("comma", "plus"),
                                "comma = best mu offspring; plus = best mu of parents and offspring."),
                        ParamSpec.real("initialSigma", 0.1, 1e-6, 1, "Initial step size as a fraction of the range."),
                        ParamSpec.real("minSigma", 1e-5, 0, 1, "Lower bound of the step size."),
                        ParamSpec.choice("stepControl", List.of("self-adaptive", "one-fifth"),
                                "Per-individual log-normal self-adaptation, or Rechenberg's 1/5 success rule."),
                        ParamSpec.integer("adaptInterval", 5, 1, 10000, "Generations between 1/5-rule updates.")),
                List.of("real-vector"), REAL, false, EvolutionStrategy::new));

        register(new Descriptor("particle-swarm", "Particle Swarm Optimisation", "swarm",
                "Particles move with inertia towards their personal best and the global (or ring-neighbourhood) "
                        + "best. Real vectors only.",
                List.of(
                        ParamSpec.integer("swarmSize", 30, 2, 10000, "Number of particles."),
                        ParamSpec.real("inertia", 0.7298, 0, 2, "Inertia weight w (start value if inertiaEnd >= 0)."),
                        ParamSpec.real("inertiaEnd", -1, -1, 2, "Final inertia for a linear decrease over the budget; -1 = constant."),
                        ParamSpec.real("cognitive", 1.49618, 0, 5, "Cognitive coefficient c1 (pull towards the personal best)."),
                        ParamSpec.real("social", 1.49618, 0, 5, "Social coefficient c2 (pull towards the neighbourhood best)."),
                        ParamSpec.real("maxVelocity", 0.2, 1e-4, 1, "Velocity limit as a fraction of each variable's range."),
                        ParamSpec.choice("topology", List.of("global", "ring"), "Global best or ring (lbest, 2 neighbours)."),
                        ParamSpec.choice("update", List.of("immediate", "synchronous"),
                                "immediate: personal bests change as soon as a particle improves (one evaluation at a time); "
                                        + "synchronous: the swarm moves on the iteration-start bests and is evaluated as one "
                                        + "(parallelisable) batch.")),
                List.of("real-vector"), REAL, false, ParticleSwarm::new));

        register(new Descriptor("nsga2", "NSGA-II", "multi-objective",
                "Elitist multi-objective GA: non-dominated sorting with constrained domination, crowding-distance "
                        + "diversity, binary tournaments. Approximates the Pareto front instead of one weighted optimum.",
                List.of(
                        ParamSpec.integer("populationSize", 40, 4, 10000, "Number of individuals."),
                        CROSSOVER,
                        ParamSpec.real("crossoverRate", 0.9, 0, 1, "Probability of recombining a parent pair."),
                        MUTATION,
                        MUTATION_RATE,
                        MUTATION_SIGMA),
                List.of("random", "mutation", "crossover", "objectives"), ANY, true, Nsga2::new));

        List<ParamSpec> memetic = new ArrayList<>(generational.subList(0, generational.size() - 1));
        memetic.add(ParamSpec.choice("replacement", List.of("generational", "plus"),
                "generational = offspring replace parents except elites; plus = (mu+lambda) truncation."));
        memetic.add(ParamSpec.choice("localSearch", MemeticAlgorithm.LOCAL_SEARCHES,
                "hill-climbing / simulated-annealing in the representation's neighbourhood, or 'problem' for the "
                        + "problem's own neighbourhood (route smoothing for navigation)."));
        memetic.add(ParamSpec.real("localRate", 0.1, 0, 1, "Probability that an offspring is refined by local search."));
        memetic.add(ParamSpec.integer("localBudget", 20, 1, 100000, "Evaluations per local search (charged to the run's budget)."));
        memetic.add(ParamSpec.real("localStepSize", 0.02, 1e-4, 1.0, "Real vectors: local-search step as a fraction of each range."));
        memetic.add(ParamSpec.real("localTemperature", 0.01, 0, 10,
                "Simulated annealing: initial temperature as a fraction of max(1, |start fitness|)."));
        memetic.add(ParamSpec.choice("learning", MemeticAlgorithm.LEARNING,
                "lamarckian: the improved solution replaces the individual; baldwinian: only its fitness is inherited."));
        memetic.add(NEIGHBOURHOOD);
        register(new Descriptor("memetic-algorithm", "Memetic Algorithm", "hybrid",
                "Genetic algorithm whose offspring are refined by a short hill-climbing, simulated-annealing or "
                        + "problem-specific local search; Lamarckian or Baldwinian learning. Local evaluations count against the budget.",
                memetic, List.of("random", "mutation", "crossover", "neighbourhood"), ANY, false, MemeticAlgorithm::new));

        register(new Descriptor("novelty-search", "Novelty Search", "diversity",
                "Rewards new behaviour instead of fitness: k-nearest-neighbour novelty over behaviour descriptors, an "
                        + "archive with an adaptive threshold, optional blending with fitness.",
                List.of(
                        ParamSpec.integer("populationSize", 30, 2, 10000, "Individuals kept per generation."),
                        ParamSpec.integer("offspring", 30, 1, 100000, "Children per generation."),
                        ParamSpec.integer("k", 10, 1, 1000, "Nearest neighbours in the novelty estimate."),
                        ParamSpec.real("archiveThreshold", 0.05, 0, 100, "Initial novelty needed to enter the archive (adapts)."),
                        ParamSpec.integer("addsPerGeneration", 4, 0, 10000, "More additions than this raise the threshold by 20%."),
                        ParamSpec.integer("maxArchive", 1000, 1, 1000000, "Archive capacity (oldest entries are dropped)."),
                        ParamSpec.real("fitnessWeight", 0.0, 0, 1, "Weight of the fitness rank in the score; 0 = pure novelty."),
                        ParamSpec.integer("tournamentSize", 3, 1, 100, "Tournament size on the score."),
                        CROSSOVER,
                        ParamSpec.real("crossoverRate", 0.8, 0, 1, "Probability of recombining a parent pair."),
                        MUTATION,
                        MUTATION_RATE,
                        MUTATION_SIGMA),
                List.of("random", "mutation", "crossover", "behaviour"), ANY, false, NoveltySearch::new));

        List<ParamSpec> island = new ArrayList<>(gaParams.subList(1, gaParams.size()));
        island.add(0, ParamSpec.integer("islands", 4, 1, 1000, "Number of islands (sub-populations)."));
        island.add(1, ParamSpec.integer("islandSize", 25, 2, 10000, "Individuals per island."));
        island.add(2, ParamSpec.integer("migrationInterval", 10, 0, 100000, "Generations between migrations; 0 = isolated islands."));
        island.add(3, ParamSpec.integer("migrants", 2, 0, 1000, "Individuals each island sends per migration."));
        island.add(4, ParamSpec.choice("topology", List.of("ring", "complete"), "Ring: to the next island; complete: to all others."));
        island.add(5, ParamSpec.choice("migrantSelection", List.of("best", "random"), "Which individuals emigrate."));
        island.add(ParamSpec.integer("elitism", 2, 0, 1000, "Best parents copied unchanged on each island."));
        register(new Descriptor("island-ga", "Island-Model GA", "parallel",
                "Several GA sub-populations evolve side by side and exchange migrants periodically; offspring of all "
                        + "islands are evaluated as one parallelisable batch.",
                island, List.of("random", "mutation", "crossover"), ANY, false, IslandGeneticAlgorithm::new));
    }

    private AlgorithmRegistry() {
    }

    @SafeVarargs
    private static List<String> union(String first, List<String>... lists) {
        List<String> out = new ArrayList<>();
        out.add(first);
        for (List<String> list : lists) {
            for (String s : list) {
                if (!out.contains(s)) {
                    out.add(s);
                }
            }
        }
        return out;
    }

    private static void register(Descriptor descriptor) {
        DESCRIPTORS.put(descriptor.id(), descriptor);
    }

    public static Descriptor get(String id) {
        Descriptor d = DESCRIPTORS.get(id);
        if (d == null) {
            throw new IllegalArgumentException("unknown algorithm '" + id + "', available: " + DESCRIPTORS.keySet());
        }
        return d;
    }

    public static List<Descriptor> all() {
        return new ArrayList<>(DESCRIPTORS.values());
    }

    public static SearchAlgorithm create(String id, Params params) {
        return get(id).create(params);
    }
}
