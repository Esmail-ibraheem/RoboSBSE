package org.robosbse.problems;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geometry.Vec2;
import org.robosbse.planning.DistanceField;
import org.robosbse.planning.GridPlanner;
import org.robosbse.planning.OccupancyGrid;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Controller parameter optimisation: tune the gains and thresholds of an actual controller so that
 * it performs well across <em>several</em> environments.
 *
 * <p><b>Representation.</b> A real vector of the selected controller parameters, each bounded by a
 * documented tuning range ({@link #RANGES}).
 *
 * <p><b>Evaluation.</b> The controller, configured with the candidate parameters, drives one real
 * simulated episode per training map: route-following controllers follow the map's smoothed A*
 * reference route, the reactive controller drives map-free to the goal. Per episode
 * {@code cost = objective} on success and {@code 1000 + 10 * D_geo(end)} on failure; the fitness is
 * the mean (or the worst case) over episodes. Validation maps are never used during search; they
 * measure how the tuned parameters generalise.
 *
 * <p><b>Objectives.</b> {@code (mean time, mean energy, mean risk)}; feasible = every training
 * episode succeeds; constraint violation = sum over failed episodes of {@code 1 + D_geo(end)/D_geo(start)}.
 */
public final class ControllerTuningProblem implements Problem<RealVector> {

    public static final String TYPE = "controller-tuning";
    public static final double FAILURE_COST = 1000;

    /** Tuning ranges per parameter name. */
    public static final Map<String, double[]> RANGES = new LinkedHashMap<>();

    static {
        RANGES.put("kHeading", new double[] {0.2, 8});
        RANGES.put("cruiseSpeed", new double[] {0.3, 1.0});
        RANGES.put("switchRadius", new double[] {0.1, 1.0});
        RANGES.put("turnInPlaceAngle", new double[] {0.2, 2.5});
        RANGES.put("approachGain", new double[] {0.2, 3.0});
        RANGES.put("kp", new double[] {0.2, 8});
        RANGES.put("ki", new double[] {0, 1});
        RANGES.put("kd", new double[] {0, 1});
        RANGES.put("lookahead", new double[] {0.2, 2.0});
        RANGES.put("speedLimit", new double[] {0.3, 1.0});
        RANGES.put("margin", new double[] {0.05, 0.5});
        RANGES.put("freeDistance", new double[] {0.4, 2.0});
        RANGES.put("decelerationFraction", new double[] {0.3, 1.0});
    }

    public static List<String> defaultParameters(String controller) {
        return switch (controller) {
            case "waypoint-follower" -> List.of("kHeading", "cruiseSpeed", "switchRadius", "turnInPlaceAngle", "approachGain");
            case "pure-pursuit" -> List.of("lookahead", "cruiseSpeed", "approachGain");
            case "reactive" -> List.of("speedLimit", "margin", "kHeading", "freeDistance", "approachGain");
            default -> List.of("kp", "ki", "kd", "cruiseSpeed", "switchRadius", "turnInPlaceAngle");
        };
    }

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.choice("controller", List.of("pid-heading", "waypoint-follower", "pure-pursuit", "reactive"),
                    "Controller whose parameters are tuned."),
            ParamSpec.text("maps", "obstacle-course,warehouse,empty-field", "Training maps (comma-separated names or files)."),
            ParamSpec.text("validationMaps", "maze", "Held-out maps for measuring generalisation (may be empty)."),
            ParamSpec.text("parameters", "default", "Parameters to tune (comma list) or 'default' for the controller's set."),
            ParamSpec.choice("objective", List.of("time", "energy", "weighted"), "Per-episode objective."),
            ParamSpec.real("weightTime", 1.0, 0, 1000, "Weighted objective: weight of travel time (s)."),
            ParamSpec.real("weightEnergy", 0.1, 0, 1000, "Weighted objective: weight of energy (J)."),
            ParamSpec.real("weightRisk", 0.0, 0, 100000, "Weighted objective: weight of collision risk."),
            ParamSpec.real("weightSmoothness", 0.0, 0, 1000, "Weighted objective: weight of angular-velocity variation."),
            ParamSpec.choice("aggregate", List.of("mean", "max"), "Aggregate over training maps: mean or worst case."),
            ParamSpec.integer("maxSteps", 2400, 10, 1_000_000, "Simulation step limit per episode (dt = 0.05 s)."));

    /** One environment the controller is evaluated on. */
    public record Episode(String name, boolean validation, WorldMap map, World world, List<Vec2> route,
                          DistanceField field, double startDistance) {
    }

    private final String controllerId;
    private final Params baseParams;
    private final List<String> names;
    private final RealVectorSpace space;
    private final List<Episode> training;
    private final List<Episode> validation;
    private final RobotSpec robot;
    private final EpisodeConfig config;
    private final String objective;
    private final double weightTime;
    private final double weightEnergy;
    private final double weightRisk;
    private final double weightSmoothness;
    private final boolean worstCase;

    public ControllerTuningProblem(List<WorldMap> trainingMaps, List<WorldMap> validationMaps, Params params,
                                   Params controllerParams, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid tuning parameters: " + String.join("; ", errors));
        }
        if (trainingMaps.isEmpty()) {
            throw new IllegalArgumentException("controller tuning needs at least one training map");
        }
        this.controllerId = params.getString("controller", "pid-heading");
        this.baseParams = controllerParams;
        this.robot = robot;
        this.config = EpisodeConfig.defaults().withMaxSteps(params.getInt("maxSteps", 2400)).withAutoComplete(true);
        this.objective = params.getString("objective", "time");
        this.weightTime = params.getDouble("weightTime", 1.0);
        this.weightEnergy = params.getDouble("weightEnergy", 0.1);
        this.weightRisk = params.getDouble("weightRisk", 0.0);
        this.weightSmoothness = params.getDouble("weightSmoothness", 0.0);
        this.worstCase = params.getString("aggregate", "mean").equals("max");
        String spec = params.getString("parameters", "default");
        this.names = spec.equals("default") ? defaultParameters(controllerId) : List.of(spec.split("\\s*,\\s*"));
        double[] lo = new double[names.size()];
        double[] hi = new double[names.size()];
        for (int i = 0; i < names.size(); i++) {
            double[] range = RANGES.get(names.get(i));
            if (range == null) {
                throw new IllegalArgumentException("no tuning range for parameter '" + names.get(i) + "', known: " + RANGES.keySet());
            }
            lo[i] = range[0];
            hi[i] = range[1];
        }
        this.space = new RealVectorSpace(lo, hi, names, 1);
        boolean reactive = "reactive".equals(ControllerRegistry.get(controllerId).kind());
        this.training = episodes(trainingMaps, false, reactive);
        this.validation = episodes(validationMaps, true, reactive);
    }

    private List<Episode> episodes(List<WorldMap> maps, boolean isValidation, boolean reactive) {
        List<Episode> out = new ArrayList<>();
        for (WorldMap map : maps) {
            World world = new World(map);
            List<Vec2> route = List.of(map.goal());
            if (!reactive) {
                GridPlanner.Plan plan = GridPlanner.astar(new OccupancyGrid(world, 0.1, robot.radius() + 0.1),
                        map.start().position(), map.goal());
                if (!plan.found()) {
                    throw new IllegalArgumentException("map '" + map.name() + "' has no A* route for the robot");
                }
                route = plan.waypoints();
            }
            DistanceField field = new DistanceField(new OccupancyGrid(world, 0.1, robot.radius()), map.goal());
            double start = Math.max(1e-6, field.distanceFrom(map.start().x(), map.start().y()));
            out.add(new Episode(map.name(), isValidation, map, world, route, field, start));
        }
        return out;
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    public List<Episode> training() {
        return training;
    }

    public List<Episode> validation() {
        return validation;
    }

    public List<String> names() {
        return names;
    }

    public String controllerId() {
        return controllerId;
    }

    public Params controllerParams(RealVector genome) {
        Params p = baseParams;
        for (int i = 0; i < names.size(); i++) {
            p = p.with(names.get(i), genome.get(i));
        }
        return p;
    }

    /** The controller's default parameter values (clamped into the tuning ranges) as a genome. */
    public RealVector defaults() {
        double[] v = new double[names.size()];
        ControllerRegistry.Descriptor d = ControllerRegistry.get(controllerId);
        for (int i = 0; i < names.size(); i++) {
            double value = space.lower(i);
            for (ParamSpec spec : d.params()) {
                if (spec.name().equals(names.get(i))) {
                    value = ((Number) spec.defaultValue()).doubleValue();
                }
            }
            v[i] = space.clamp(i, baseParams.getDouble(names.get(i), value));
        }
        return new RealVector(v);
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("time", "energy", "risk");
    }

    @Override
    public double[] referencePoint() {
        return new double[] {config.maxSteps() * config.dt(), robot.batteryCapacity(), 1.0};
    }

    public EpisodeResult simulate(Episode e, Params controllerParams, EpisodeConfig cfg) {
        ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), e.map().goal(), e.map().goalTolerance(), e.route(),
                controllerParams);
        return Simulator.run(e.world(), robot, e.map().start(), e.map().goal(), e.map().goalTolerance(),
                ControllerRegistry.create(controllerId), setup, cfg);
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluateOn(training, genome);
    }

    /** Aggregated evaluation of a genome on a set of episodes (training or validation). */
    public Evaluation evaluateOn(List<Episode> episodes, RealVector genome) {
        Params cp = controllerParams(genome);
        double sum = 0;
        double worst = Double.NEGATIVE_INFINITY;
        double time = 0;
        double energy = 0;
        double risk = 0;
        double clearance = 0;
        double smoothness = 0;
        double violation = 0;
        int successes = 0;
        Map<String, Double> perEpisode = new LinkedHashMap<>();
        double[] behaviour = new double[2 * episodes.size()];
        int index = 0;
        for (Episode e : episodes) {
            EpisodeResult r = simulate(e, cp, config);
            behaviour[index++] = (double) r.steps() / config.maxSteps();
            behaviour[index++] = Math.max(0, Math.min(1, r.minObstacleClearance()));
            double episodeRisk = NavigationProblem.risk(r.minObstacleClearance());
            double cost;
            if (r.success()) {
                successes++;
                cost = switch (objective) {
                    case "energy" -> r.energy();
                    case "weighted" -> weightTime * r.time() + weightEnergy * r.energy() + weightRisk * episodeRisk
                            + weightSmoothness * r.smoothness();
                    default -> r.time();
                };
            } else {
                double remaining = e.field().distanceFrom(r.finalPose().x(), r.finalPose().y());
                cost = FAILURE_COST + 10 * remaining;
                violation += 1 + remaining / e.startDistance();
            }
            sum += cost;
            worst = Math.max(worst, cost);
            time += r.time() / episodes.size();
            energy += r.energy() / episodes.size();
            risk += episodeRisk / episodes.size();
            clearance += Math.min(r.minObstacleClearance(), 10) / episodes.size();
            smoothness += r.smoothness() / episodes.size();
            perEpisode.put("cost." + e.name(), cost);
        }
        double fitness = worstCase ? worst : sum / Math.max(1, episodes.size());
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("successRate", episodes.isEmpty() ? 0 : (double) successes / episodes.size());
        metrics.put("meanTime", time);
        metrics.put("meanEnergy", energy);
        metrics.put("meanRisk", risk);
        metrics.put("meanMinClearance", clearance);
        metrics.put("meanSmoothness", smoothness);
        metrics.put("worstCost", worst);
        metrics.putAll(perEpisode);
        return new Evaluation(fitness, violation, metrics, new double[] {time, energy, risk}, behaviour);
    }

    @Override
    public Object view(RealVector genome) {
        Map<String, Object> view = new LinkedHashMap<>();
        Map<String, Double> values = new LinkedHashMap<>();
        for (int i = 0; i < names.size(); i++) {
            values.put(names.get(i), genome.get(i));
        }
        view.put("controller", controllerId);
        view.put("params", values);
        view.put("genome", genome.toArray());
        return view;
    }

    /** Recorded episodes on every training and validation map, for replay. */
    public List<Map<String, Object>> replay(RealVector genome) {
        Params cp = controllerParams(genome);
        List<Map<String, Object>> out = new ArrayList<>();
        List<Episode> all = new ArrayList<>(training);
        all.addAll(validation);
        for (Episode e : all) {
            EpisodeResult r = simulate(e, cp, config.recording(2, "reactive".equals(controllerId)));
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("map", e.map());
            m.put("split", e.validation() ? "validation" : "training");
            m.put("outcome", r.outcome().name());
            m.put("metrics", r.metrics());
            List<double[]> route = new ArrayList<>();
            route.add(new double[] {e.map().start().x(), e.map().start().y()});
            for (Vec2 p : e.route()) {
                route.add(new double[] {p.x(), p.y()});
            }
            m.put("route", route);
            m.put("trajectory", r.trajectory());
            out.add(m);
        }
        return out;
    }
}
