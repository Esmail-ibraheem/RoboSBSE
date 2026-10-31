package org.robosbse.geneticprogramming;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.planning.DistanceField;
import org.robosbse.planning.OccupancyGrid;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Genetic programming of reactive controllers: evolve a {@link Program} that drives the robot from start
 * to goal on every training map using only its range sensor and the goal direction (no map, no planner).
 *
 * <p><b>Fitness (minimised).</b> Per episode: the travel time on success; otherwise
 * {@code 1000 + 10 * remaining geodesic distance} (a collision, timeout or deadlock ends the episode).
 * The fitness is the mean over the training episodes plus parsimony pressure
 * {@code parsimony * size} against bloat. Objectives for multi-objective search: (mean episode cost,
 * program size). <b>Generalisation</b> is measured on separate test maps that the search never sees,
 * and the hand-written reactive controller on the same episodes is the conventional reference.
 */
public final class ControllerSynthesisProblem implements Problem<Program> {

    public static final String TYPE = "controller-synthesis";
    static final double FAILURE_COST = 1000;

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("maps", "empty-field,obstacle-course,generated:scatter:1,generated:scatter:2",
                    "Training maps (builtin names, files or generated:scatter:<seed>)."),
            ParamSpec.text("testMaps", "warehouse,generated:scatter:101,generated:scatter:102,generated:scatter:103",
                    "Unseen test maps: never used by the search, only to measure generalisation."),
            ParamSpec.integer("maxSteps", 1800, 10, 1_000_000, "Simulation step limit per episode (dt = 0.05 s)."),
            ParamSpec.integer("maxSize", 40, 1, 500, "Maximum program size (nodes)."),
            ParamSpec.integer("maxDepth", 7, 1, 30, "Maximum program depth."),
            ParamSpec.integer("minInitDepth", 2, 1, 30, "Smallest depth of initial programs (ramped half-and-half)."),
            ParamSpec.integer("maxInitDepth", 4, 1, 30, "Largest depth of initial programs."),
            ParamSpec.real("parsimony", 0.05, 0, 100, "Fitness penalty per program node (bloat control)."),
            ParamSpec.integer("stepBudget", 500, 1, 1_000_000, "Interpreter node evaluations allowed per control tick."));

    /** One evaluation episode on a map. */
    public record Episode(String name, String split, WorldMap map, World world, DistanceField field, double startDistance) {
    }

    private final ProgramSpace space;
    private final List<Episode> training;
    private final List<Episode> test;
    private final RobotSpec robot;
    private final EpisodeConfig config;
    private final double parsimony;
    private final int stepBudget;

    public ControllerSynthesisProblem(List<WorldMap> trainingMaps, List<WorldMap> testMaps, Params params, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid controller-synthesis parameters: " + String.join("; ", errors));
        }
        if (trainingMaps.isEmpty()) {
            throw new IllegalArgumentException("controller synthesis needs at least one training map");
        }
        this.robot = robot;
        this.space = new ProgramSpace(params.getInt("maxSize", 40), params.getInt("maxDepth", 7),
                params.getInt("minInitDepth", 2), params.getInt("maxInitDepth", 4));
        this.config = EpisodeConfig.defaults().withMaxSteps(params.getInt("maxSteps", 1800)).withAutoComplete(true);
        this.parsimony = params.getDouble("parsimony", 0.05);
        this.stepBudget = params.getInt("stepBudget", 500);
        this.training = episodes(trainingMaps, "training");
        this.test = episodes(testMaps, "test");
    }

    private List<Episode> episodes(List<WorldMap> maps, String split) {
        List<Episode> out = new ArrayList<>();
        for (WorldMap map : maps) {
            World world = new World(map);
            DistanceField field = new DistanceField(new OccupancyGrid(world, 0.1, robot.radius()), map.goal());
            double start = field.distanceFrom(map.start().x(), map.start().y());
            if (!Double.isFinite(start)) {
                throw new IllegalArgumentException("map '" + map.name() + "' has no route from start to goal");
            }
            out.add(new Episode(map.name(), split, map, world, field, Math.max(1e-6, start)));
        }
        return out;
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public ProgramSpace space() {
        return space;
    }

    public List<Episode> training() {
        return training;
    }

    public List<Episode> test() {
        return test;
    }

    public EpisodeResult simulate(Episode e, Controller controller, EpisodeConfig cfg) {
        ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), e.map().goal(), e.map().goalTolerance(),
                List.of(e.map().goal()), Params.empty());
        return Simulator.run(e.world(), robot, e.map().start(), e.map().goal(), e.map().goalTolerance(), controller,
                setup, cfg);
    }

    /** Cost of one episode: travel time on success, else 1000 + 10 x remaining geodesic distance. */
    double cost(Episode e, EpisodeResult r) {
        if (r.success()) {
            return r.time();
        }
        double remaining = e.field().distanceFrom(r.finalPose().x(), r.finalPose().y());
        return FAILURE_COST + 10 * (Double.isFinite(remaining) ? remaining : e.startDistance());
    }

    @Override
    public Evaluation evaluate(Program program) {
        return evaluateOn(training, program, true);
    }

    /** Mean episode cost on {@code episodes}; with {@code withParsimony} the size penalty is added. */
    public Evaluation evaluateOn(List<Episode> episodes, Program program, boolean withParsimony) {
        return evaluateWith(episodes, () -> new ProgramController(program, stepBudget),
                withParsimony ? parsimony * program.size() : 0, program.size(), program.depth());
    }

    /** The hand-written reactive controller (default parameters) on the same episodes: the reference. */
    public Evaluation evaluateReference(List<Episode> episodes) {
        return evaluateWith(episodes, () -> ControllerRegistry.create("reactive"), 0, 0, 0);
    }

    private Evaluation evaluateWith(List<Episode> episodes, java.util.function.Supplier<Controller> factory,
                                    double penalty, int size, int depth) {
        double sum = 0;
        double time = 0;
        int successes = 0;
        double[] behaviour = new double[2 * episodes.size()];
        Map<String, Double> perEpisode = new LinkedHashMap<>();
        int k = 0;
        for (Episode e : episodes) {
            EpisodeResult r = simulate(e, factory.get(), config);
            double cost = cost(e, r);
            sum += cost;
            if (r.success()) {
                successes++;
                time += r.time();
            }
            double remaining = e.field().distanceFrom(r.finalPose().x(), r.finalPose().y());
            behaviour[k++] = Math.max(0, Math.min(1, 1 - (Double.isFinite(remaining) ? remaining : e.startDistance())
                    / e.startDistance()));
            behaviour[k++] = (double) r.steps() / config.maxSteps();
            perEpisode.put("cost." + e.name(), cost);
        }
        double mean = sum / Math.max(1, episodes.size());
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("successRate", episodes.isEmpty() ? 0 : (double) successes / episodes.size());
        metrics.put("meanCost", mean);
        metrics.put("meanTimeOfSuccesses", successes == 0 ? Double.NaN : time / successes);
        metrics.put("size", (double) size);
        metrics.put("depth", (double) depth);
        metrics.putAll(perEpisode);
        metrics.values().removeIf(v -> !Double.isFinite(v));
        int failures = episodes.size() - successes;
        return new Evaluation(mean + penalty, failures, metrics, new double[] {mean, size}, behaviour);
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        return evaluation.metric("successRate", 0) >= 1.0;
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("meanCost", "size");
    }

    @Override
    public double[] referencePoint() {
        double worst = 0;
        for (Episode e : training) {
            worst = Math.max(worst, FAILURE_COST + 10 * e.startDistance());
        }
        return new double[] {worst * 1.01, space.maxSize() + 1};
    }

    @Override
    public Object view(Program program) {
        Map<String, Object> view = new LinkedHashMap<>();
        view.put("program", program.toString());
        view.put("pretty", program.pretty());
        view.put("size", program.size());
        view.put("depth", program.depth());
        view.put("genome", program.toString());
        return view;
    }

    /** Recorded episodes on training and test maps, in the tuning replay format. */
    public List<Map<String, Object>> replay(Program program) {
        List<Map<String, Object>> out = new ArrayList<>();
        List<Episode> all = new ArrayList<>(training);
        all.addAll(test);
        for (Episode e : all) {
            EpisodeResult r = simulate(e, new ProgramController(program, stepBudget), config.recording(2, true));
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("map", e.map());
            m.put("split", e.split());
            m.put("outcome", r.outcome().name());
            m.put("metrics", r.metrics());
            m.put("cost", cost(e, r));
            m.put("trajectory", r.trajectory());
            out.add(m);
        }
        return out;
    }

    public Program parse(String text) {
        return Program.parse(text, space.maxSize(), space.maxDepth());
    }
}
