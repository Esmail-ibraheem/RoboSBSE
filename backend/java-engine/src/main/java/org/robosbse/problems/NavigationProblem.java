package org.robosbse.problems;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Evaluation;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.Rng;
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
 * Route optimisation: find intermediate waypoints that let a route-following controller drive the
 * robot from the start pose to the goal without collision, minimising an objective measured by
 * simulating the robot.
 *
 * <p><b>Representation.</b> {@code K} waypoints encoded as a real vector
 * {@code (x1, y1, ..., xK, yK)} inside the world (block size 2, so crossover never splits a
 * waypoint). The executed route is start → w1 → ... → wK → goal.
 *
 * <p><b>Fitness (minimised).</b> Every candidate is executed by the simulator with the configured
 * controller. With objective {@code o} (time, energy, travelled length, or a weighted sum):
 * <pre>
 *   success:  f = o + 50 * V
 *   failure:  f = 10000 + 10 * D_geo(final position) + 50 * V
 * </pre>
 * where {@code V} is the constraint-violation integral of the planned polyline,
 * {@code integral of max(0, r + 0.05 - clearance(p)) ds}, which penalises route segments that pass
 * through or too close to obstacles in proportion to depth and length, and {@code D_geo} is the
 * geodesic (obstacle-aware) distance from where the robot stopped to the goal. The constant offset
 * makes every successful route better than every failed one; among failures, the progress term
 * gives the search a gradient.
 *
 * <p><b>Repair.</b> With {@code repair = true}, waypoints that lie inside inflated obstacles are
 * pushed out along the clearance gradient before evaluation (Lamarckian: the repaired genome is kept).
 *
 * <p><b>Objectives (multi-objective algorithms).</b> {@code (time, energy, risk)}, all minimised, with
 * collision risk {@code risk = exp(-max(0, minClearance) / 0.2 m)} (1 when touching an obstacle,
 * 0.08 at 0.5 m clearance). A route is feasible only if the robot arrives and the planned polyline
 * respects the inflated obstacles; otherwise the constraint violation is
 * {@code V + 1 + D_geo(end) / D_geo(start)} (failures closer to the goal violate less).
 * Hypervolume reference point: {@code (episode time limit, battery capacity, 1)} unless
 * {@code referenceTime} / {@code referenceEnergy} set tighter values.
 */
public final class NavigationProblem implements Problem<RealVector> {

    public static final String TYPE = "navigation";
    public static final double FAILURE_OFFSET = 10_000;
    public static final double VIOLATION_WEIGHT = 50;
    public static final double PROGRESS_WEIGHT = 10;
    /** Penalty per second spent in a restricted zone (dynamic maps only). */
    public static final double RESTRICTED_WEIGHT = 20;

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.integer("waypoints", 5, 1, 40, "Number of intermediate waypoints K (genome length 2K)."),
            ParamSpec.choice("controller", List.of("waypoint-follower", "pid-heading", "pure-pursuit"),
                    "Route-following controller that executes candidate routes."),
            ParamSpec.choice("objective", List.of("time", "energy", "length", "weighted"), "Objective to minimise."),
            ParamSpec.real("weightTime", 1.0, 0, 1000, "Weighted objective: weight of travel time (s)."),
            ParamSpec.real("weightEnergy", 0.1, 0, 1000, "Weighted objective: weight of energy (J)."),
            ParamSpec.real("weightLength", 0.0, 0, 1000, "Weighted objective: weight of travelled length (m)."),
            ParamSpec.real("weightSmoothness", 0.0, 0, 1000, "Weighted objective: weight of angular-velocity variation."),
            ParamSpec.real("weightRisk", 0.0, 0, 100000, "Weighted objective: weight of collision risk exp(-clearance/0.2)."),
            ParamSpec.real("referenceTime", 0, 0, 1e6, "Hypervolume reference for time (s); 0 = the episode time limit."),
            ParamSpec.real("referenceEnergy", 0, 0, 1e9, "Hypervolume reference for energy (J); 0 = the battery capacity."),
            ParamSpec.integer("maxSteps", 2400, 10, 1_000_000, "Simulation step limit per evaluation (dt = 0.05 s)."),
            ParamSpec.bool("repair", true, "Push waypoints out of inflated obstacles before evaluation."),
            ParamSpec.choice("progress", List.of("geodesic", "euclidean"),
                    "Distance-to-goal measure used in the failure penalty."));

    private final WorldMap map;
    private final World world;
    private final RobotSpec robot;
    private final int waypoints;
    private final String controllerId;
    private final Params controllerParams;
    private final String objective;
    private final double weightTime;
    private final double weightEnergy;
    private final double weightLength;
    private final double weightSmoothness;
    private final double weightRisk;
    private final double startDistance;
    private final double referenceTime;
    private final double referenceEnergy;
    private final EpisodeConfig episode;
    private final boolean repair;
    private final boolean geodesic;
    private final RealVectorSpace space;
    private final OccupancyGrid clearanceGrid;
    private final DistanceField distanceField;
    private final double inflation;

    public NavigationProblem(WorldMap map, Params params, Params controllerParams, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid navigation parameters: " + String.join("; ", errors));
        }
        this.map = map;
        this.world = new World(map);
        this.robot = robot;
        this.waypoints = params.getInt("waypoints", 5);
        this.controllerId = params.getString("controller", "waypoint-follower");
        this.controllerParams = controllerParams;
        this.objective = params.getString("objective", "time");
        this.weightTime = params.getDouble("weightTime", 1.0);
        this.weightEnergy = params.getDouble("weightEnergy", 0.1);
        this.weightLength = params.getDouble("weightLength", 0.0);
        this.weightSmoothness = params.getDouble("weightSmoothness", 0.0);
        this.weightRisk = params.getDouble("weightRisk", 0.0);
        this.referenceTime = params.getDouble("referenceTime", 0);
        this.referenceEnergy = params.getDouble("referenceEnergy", 0);
        this.episode = EpisodeConfig.defaults().withMaxSteps(params.getInt("maxSteps", 2400)).withAutoComplete(true);
        this.repair = params.getBool("repair", true);
        this.geodesic = params.getString("progress", "geodesic").equals("geodesic");
        if (!"route-following".equals(ControllerRegistry.get(controllerId).kind())) {
            throw new IllegalArgumentException("navigation needs a route-following controller, got " + controllerId);
        }
        double r = robot.radius();
        double[] lo = new double[2 * waypoints];
        double[] hi = new double[2 * waypoints];
        List<String> names = new ArrayList<>();
        for (int i = 0; i < waypoints; i++) {
            lo[2 * i] = r;
            hi[2 * i] = map.width() - r;
            lo[2 * i + 1] = r;
            hi[2 * i + 1] = map.height() - r;
            names.add("w" + (i + 1) + ".x");
            names.add("w" + (i + 1) + ".y");
        }
        this.space = new RealVectorSpace(lo, hi, names, 2);
        this.inflation = r + 0.05;
        this.clearanceGrid = new OccupancyGrid(world, 0.1, inflation);
        this.distanceField = new DistanceField(new OccupancyGrid(world, 0.1, r), map.goal());
        this.startDistance = Math.max(1e-6, distanceField.distanceFrom(map.start().x(), map.start().y()));
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("time", "energy", "risk");
    }

    @Override
    public double[] referencePoint() {
        return new double[] {
                referenceTime > 0 ? referenceTime : episode.maxSteps() * episode.dt(),
                referenceEnergy > 0 ? referenceEnergy : robot.batteryCapacity() * episode.initialBattery(),
                1.0};
    }

    /** Collision risk in (0, 1]: exp(-clearance / 0.2 m), 1 at contact. */
    public static double risk(double minClearance) {
        return Math.exp(-Math.max(0, minClearance) / 0.2);
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    public WorldMap map() {
        return map;
    }

    public RobotSpec robot() {
        return robot;
    }

    /** Waypoints encoded by the genome followed by the goal: the route handed to the controller. */
    public List<Vec2> route(RealVector genome) {
        List<Vec2> route = new ArrayList<>(waypoints + 1);
        for (int i = 0; i < waypoints; i++) {
            route.add(new Vec2(genome.get(2 * i), genome.get(2 * i + 1)));
        }
        route.add(map.goal());
        return route;
    }

    @Override
    public RealVector repair(RealVector genome) {
        if (!repair) {
            return genome;
        }
        double[] v = genome.toArray();
        boolean changed = false;
        for (int i = 0; i < waypoints; i++) {
            double x = v[2 * i];
            double y = v[2 * i + 1];
            for (int iter = 0; iter < 6; iter++) {
                double c = clearanceGrid.clearanceAt(x, y);
                if (c >= inflation) {
                    break;
                }
                double h = 0.05;
                double gx = (clearanceGrid.clearanceAt(x + h, y) - clearanceGrid.clearanceAt(x - h, y)) / (2 * h);
                double gy = (clearanceGrid.clearanceAt(x, y + h) - clearanceGrid.clearanceAt(x, y - h)) / (2 * h);
                double norm = Math.hypot(gx, gy);
                if (norm < 1e-9) {
                    break;
                }
                double push = inflation - c + 0.02;
                x = space.clamp(2 * i, x + gx / norm * push);
                y = space.clamp(2 * i + 1, y + gy / norm * push);
                changed = true;
            }
            v[2 * i] = x;
            v[2 * i + 1] = y;
        }
        return changed ? new RealVector(v) : genome;
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluateRoute(route(genome));
    }

    /**
     * Route smoothing ("string pulling") for memetic local search: a move pulls one waypoint towards
     * the midpoint of its two neighbours on the route (the start, the other waypoints, the goal) by a
     * fraction alpha. {@code random} draws the waypoint uniformly and alpha from U(0.25, 1);
     * {@code all} lists alpha = 0.5 and alpha = 1 for every waypoint. Moves shorten the route and
     * reduce its turning; whether that is better is decided by simulation as usual.
     */
    @Override
    public Neighbourhood<RealVector> domainNeighbourhood() {
        return new Neighbourhood<>() {
            @Override
            public List<RealVector> all(RealVector genome) {
                List<RealVector> out = new ArrayList<>(2 * waypoints);
                for (int i = 0; i < waypoints; i++) {
                    out.add(smooth(genome, i, 0.5));
                    out.add(smooth(genome, i, 1.0));
                }
                return out;
            }

            @Override
            public RealVector random(RealVector genome, Rng rng) {
                return smooth(genome, rng.nextInt(waypoints), rng.uniform(0.25, 1.0));
            }
        };
    }

    /** Moves waypoint {@code i} the fraction {@code alpha} of the way to the midpoint of its neighbours. */
    RealVector smooth(RealVector genome, int i, double alpha) {
        double[] v = genome.toArray();
        Vec2 prev = i == 0 ? map.start().position() : new Vec2(v[2 * i - 2], v[2 * i - 1]);
        Vec2 next = i == waypoints - 1 ? map.goal() : new Vec2(v[2 * i + 2], v[2 * i + 3]);
        double midX = (prev.x() + next.x()) / 2;
        double midY = (prev.y() + next.y()) / 2;
        v[2 * i] += alpha * (midX - v[2 * i]);
        v[2 * i + 1] += alpha * (midY - v[2 * i + 1]);
        return new RealVector(v);
    }

    /** Evaluates any route ending at the goal (also used for the A* reference route). */
    public Evaluation evaluateRoute(List<Vec2> route) {
        EpisodeResult result = simulate(route, episode);
        double violation = violation(route);
        double fitness;
        double objectiveValue = objective(result);
        double constraint = violation + result.restrictedTime();
        if (result.success()) {
            fitness = objectiveValue + VIOLATION_WEIGHT * violation + RESTRICTED_WEIGHT * result.restrictedTime();
        } else {
            Vec2 end = result.finalPose().position();
            double remaining = geodesic ? distanceField.distanceFrom(end.x(), end.y()) : end.distanceTo(map.goal());
            fitness = FAILURE_OFFSET + PROGRESS_WEIGHT * remaining + VIOLATION_WEIGHT * violation
                    + RESTRICTED_WEIGHT * result.restrictedTime();
            constraint += 1 + distanceField.distanceFrom(end.x(), end.y()) / startDistance;
        }
        double risk = risk(result.minObstacleClearance());
        Map<String, Double> metrics = new LinkedHashMap<>(result.metrics());
        metrics.put("objective", objectiveValue);
        metrics.put("violation", violation);
        metrics.put("plannedLength", GridPlanner.polylineLength(map.start().position(), route));
        metrics.put("outcome", (double) result.outcome().ordinal());
        metrics.put("risk", risk);
        double[] objectives = {result.time(), result.energy(), risk};
        return new Evaluation(fitness, constraint, metrics, objectives, result.behaviour(map.width(), map.height()));
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        return evaluation.metric("success", 0) > 0;
    }

    private double objective(EpisodeResult r) {
        return switch (objective) {
            case "energy" -> r.energy();
            case "length" -> r.distance();
            case "weighted" -> weightTime * r.time() + weightEnergy * r.energy() + weightLength * r.distance()
                    + weightSmoothness * r.smoothness() + weightRisk * risk(r.minObstacleClearance());
            default -> r.time();
        };
    }

    /** Depth-weighted length of the planned polyline inside the inflated obstacles. */
    public double violation(List<Vec2> route) {
        double total = 0;
        Vec2 previous = map.start().position();
        double ds = 0.05;
        for (Vec2 p : route) {
            double length = previous.distanceTo(p);
            int n = Math.max(1, (int) Math.ceil(length / ds));
            double step = length / n;
            for (int i = 0; i < n; i++) {
                double t = (i + 0.5) / n;
                double x = previous.x() + t * (p.x() - previous.x());
                double y = previous.y() + t * (p.y() - previous.y());
                double deficit = inflation - clearanceGrid.clearanceAt(x, y);
                if (deficit > 0) {
                    total += deficit * step;
                }
            }
            previous = p;
        }
        return total;
    }

    public EpisodeResult simulate(List<Vec2> route, EpisodeConfig config) {
        ControllerSetup setup = new ControllerSetup(robot, config.dt(), map.goal(), map.goalTolerance(), route,
                controllerParams);
        return Simulator.run(world, robot, map.start(), map.goal(), map.goalTolerance(),
                ControllerRegistry.create(controllerId), setup, config);
    }

    /** Simulation of a route with a recorded trajectory, for replay. */
    public EpisodeResult replay(List<Vec2> route) {
        return simulate(route, episode.recording(2, false));
    }

    @Override
    public Object view(RealVector genome) {
        Map<String, Object> view = new LinkedHashMap<>();
        List<double[]> points = new ArrayList<>();
        for (Vec2 p : route(genome)) {
            points.add(new double[] {p.x(), p.y()});
        }
        view.put("route", points);
        view.put("genome", genome.toArray());
        return view;
    }

    /** A* and Dijkstra reference plans, executed with the same controller and fitness function. */
    public Map<String, Object> reference() {
        OccupancyGrid planningGrid = new OccupancyGrid(world, 0.1, robot.radius() + 0.1);
        Map<String, Object> out = new LinkedHashMap<>();
        for (GridPlanner.Plan plan : List.of(
                GridPlanner.astar(planningGrid, map.start().position(), map.goal()),
                GridPlanner.dijkstra(planningGrid, map.start().position(), map.goal()))) {
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("found", plan.found());
            entry.put("expansions", plan.expansions());
            entry.put("planningMicros", plan.micros());
            entry.put("gridLength", plan.gridLength());
            entry.put("smoothLength", plan.smoothLength());
            List<double[]> pts = new ArrayList<>();
            for (Vec2 p : plan.waypoints()) {
                pts.add(new double[] {p.x(), p.y()});
            }
            entry.put("route", pts);
            if (plan.found()) {
                Evaluation e = evaluateRoute(plan.waypoints());
                entry.put("fitness", e.fitness());
                entry.put("metrics", e.metrics());
                if (plan.algorithm().equals("astar")) {
                    entry.put("trajectory", replay(plan.waypoints()).trajectory());
                }
            }
            out.put(plan.algorithm(), entry);
        }
        out.put("note", "Reference planners search a 0.1 m grid; their effort is reported as node expansions, "
                + "not fitness evaluations. Their routes are executed with the same controller and scored with "
                + "the same fitness function as the metaheuristics.");
        return out;
    }
}
