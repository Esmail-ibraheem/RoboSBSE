package org.robosbse.problems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.Permutation;
import org.robosbse.core.representation.PermutationSpace;
import org.robosbse.geometry.Vec2;
import org.robosbse.planning.DistanceField;
import org.robosbse.planning.GridPlanner;
import org.robosbse.planning.OccupancyGrid;
import org.robosbse.problems.warehouse.TaskSet;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.Trajectory;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Warehouse task scheduling: order the pickups and deliveries of a set of tasks for one robot with
 * limited carrying capacity, task dependencies, deadlines and a battery that must be recharged.
 *
 * <p><b>Representation.</b> A permutation of the 2n stops (pickup and delivery of every task) used
 * as a <em>priority list</em>. The decoder repeatedly takes the first stop in the list that is
 * feasible now: a pickup needs free capacity and all of the task's prerequisite tasks delivered; a
 * delivery needs its pickup done. Before every leg it checks the energy needed to reach the stop,
 * serve it and still reach the charger with the reserve left; if that is not available the robot
 * first detours to the charger and charges fully. Every decoded schedule therefore satisfies
 * precedence, capacity, dependency and battery constraints by construction; only deadlines can be
 * missed. (Different lists may decode to the same schedule — the usual price of a decoder.)
 *
 * <p><b>Cost model.</b> Travel distances are geodesic (Dijkstra on the inflated occupancy grid of
 * the map), travel time = distance / planning speed, plus {@code serviceTime} per stop and charging
 * time at {@code chargeRate}. Energy uses the robot's power model at the planning speed. Replays
 * execute the resulting mission in the simulator (leg by leg along A* routes) and report the
 * simulated time next to the modelled one.
 *
 * <p><b>Objectives.</b> {@code (makespan, total tardiness, energy)}. Hypervolume reference point:
 * 1.5 x the greedy (nearest-feasible-stop) schedule's objectives, plus 60 s of tardiness.
 */
public final class WarehouseSchedulingProblem implements Problem<Permutation> {

    public static final String TYPE = "warehouse-scheduling";
    private static final int DEPOT = 0;
    private static final int CHARGER = 1;

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("tasks", "builtin:orders-8", "Task set: builtin:orders-4, builtin:orders-8, a file or an inline object."),
            ParamSpec.choice("objective", List.of("weighted", "makespan", "tardiness", "distance", "energy"),
                    "Single objective; weighted = makespan + wT * tardiness + wD * distance + wE * energy."),
            ParamSpec.real("weightTardiness", 5.0, 0, 1e6, "Weight of total tardiness (s) in the weighted objective."),
            ParamSpec.real("weightDistance", 0.0, 0, 1e6, "Weight of travelled distance (m) in the weighted objective."),
            ParamSpec.real("weightEnergy", 0.0, 0, 1e6, "Weight of energy (J) in the weighted objective."),
            ParamSpec.bool("hardDeadlines", false, "Treat tardiness as a constraint violation (infeasible if late)."));

    /** One stop of a decoded schedule. */
    public record Visit(String label, String kind, int task, double x, double y, double arrival, double departure,
                        int load, double battery, double lateness) {
    }

    /** A decoded schedule and its measured properties. */
    public record Schedule(List<Visit> visits, double makespan, double tardiness, double distance, double energy,
                           int charges, int lateTasks, double loadedDistance) {
    }

    private final TaskSet tasks;
    private final WorldMap map;
    private final World world;
    private final RobotSpec robot;
    private final PermutationSpace space;
    private final Vec2[] points;
    private final String[] labels;
    private final double[][] distance;
    private final int[][] after;
    private final String objective;
    private final double weightTardiness;
    private final double weightDistance;
    private final double weightEnergy;
    private final boolean hardDeadlines;
    private final double capacityJ;
    private final double powerTravel;
    private final double[] reference;

    public WarehouseSchedulingProblem(TaskSet tasks, WorldMap map, Params params, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid scheduling parameters: " + String.join("; ", errors));
        }
        int n = tasks.tasks().size();
        if (n < 1) {
            throw new IllegalArgumentException("task set '" + tasks.name() + "' has no tasks");
        }
        this.tasks = tasks;
        this.map = map;
        this.world = new World(map);
        this.robot = robot;
        this.space = new PermutationSpace(2 * n);
        this.objective = params.getString("objective", "weighted");
        this.weightTardiness = params.getDouble("weightTardiness", 5.0);
        this.weightDistance = params.getDouble("weightDistance", 0.0);
        this.weightEnergy = params.getDouble("weightEnergy", 0.0);
        this.hardDeadlines = params.getBool("hardDeadlines", false);
        this.capacityJ = robot.batteryCapacity();
        this.powerTravel = robot.power(tasks.speed(), 0, 0);

        points = new Vec2[2 + 2 * n];
        labels = new String[points.length];
        points[DEPOT] = tasks.depot();
        labels[DEPOT] = "depot";
        points[CHARGER] = tasks.charger();
        labels[CHARGER] = "charger";
        Map<String, Integer> index = new HashMap<>();
        for (int k = 0; k < n; k++) {
            TaskSet.Task t = tasks.tasks().get(k);
            if (index.put(t.id(), k) != null) {
                throw new IllegalArgumentException("duplicate task id " + t.id());
            }
            points[2 + 2 * k] = t.pickup();
            labels[2 + 2 * k] = "P:" + t.id();
            points[3 + 2 * k] = t.delivery();
            labels[3 + 2 * k] = "D:" + t.id();
        }
        after = new int[n][];
        for (int k = 0; k < n; k++) {
            List<String> deps = tasks.tasks().get(k).after();
            after[k] = new int[deps.size()];
            for (int j = 0; j < deps.size(); j++) {
                Integer d = index.get(deps.get(j));
                if (d == null) {
                    throw new IllegalArgumentException("task " + tasks.tasks().get(k).id() + " depends on unknown task " + deps.get(j));
                }
                after[k][j] = d;
            }
        }
        checkAcyclic(n);

        OccupancyGrid grid = new OccupancyGrid(world, 0.1, robot.radius() + 0.1);
        distance = new double[points.length][points.length];
        for (int i = 0; i < points.length; i++) {
            DistanceField field = new DistanceField(grid, points[i]);
            for (int j = 0; j < points.length; j++) {
                if (!field.reachable(points[j].x(), points[j].y()) && i != j) {
                    throw new IllegalArgumentException(labels[j] + " is not reachable from " + labels[i]);
                }
                distance[i][j] = i == j ? 0 : field.distanceFrom(points[j].x(), points[j].y());
            }
        }
        checkEnergyFeasible(n);
        Schedule greedy = decode(greedyOrder());
        reference = new double[] {1.5 * greedy.makespan(), 1.5 * greedy.tardiness() + 60, 1.5 * greedy.energy()};
    }

    private void checkAcyclic(int n) {
        int[] indegree = new int[n];
        for (int k = 0; k < n; k++) {
            indegree[k] = after[k].length;
        }
        Deque<Integer> ready = new ArrayDeque<>();
        for (int k = 0; k < n; k++) {
            if (indegree[k] == 0) {
                ready.add(k);
            }
        }
        int seen = 0;
        while (!ready.isEmpty()) {
            int k = ready.poll();
            seen++;
            for (int j = 0; j < n; j++) {
                for (int d : after[j]) {
                    if (d == k && --indegree[j] == 0) {
                        ready.add(j);
                    }
                }
            }
        }
        if (seen < n) {
            throw new IllegalArgumentException("task dependencies contain a cycle");
        }
    }

    private void checkEnergyFeasible(int n) {
        double reserve = tasks.reserve() * capacityJ;
        for (int s = 2; s < points.length; s++) {
            double roundTrip = legEnergy(distance[CHARGER][s]) + serviceEnergy() + legEnergy(distance[s][CHARGER]);
            if (roundTrip + reserve > capacityJ) {
                throw new IllegalArgumentException(labels[s] + " cannot be served on a full battery from the charger");
            }
        }
        if (legEnergy(distance[DEPOT][CHARGER]) + reserve > tasks.initialBattery() * capacityJ) {
            throw new IllegalArgumentException("the initial battery does not even reach the charger");
        }
    }

    private double legEnergy(double metres) {
        return powerTravel * metres / tasks.speed();
    }

    private double serviceEnergy() {
        return robot.idlePower() * tasks.serviceTime();
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<Permutation> space() {
        return space;
    }

    public WorldMap map() {
        return map;
    }

    public TaskSet tasks() {
        return tasks;
    }

    /** Geodesic distance between two points (0 = depot, 1 = charger, 2 + s = stop s). */
    public double distance(int from, int to) {
        return distance[from][to];
    }

    /** Decodes a priority list into a feasible schedule (see the class comment). */
    public Schedule decode(Permutation priority) {
        int n = tasks.tasks().size();
        List<Integer> pending = new ArrayList<>(2 * n);
        for (int i = 0; i < priority.size(); i++) {
            pending.add(priority.get(i));
        }
        boolean[] picked = new boolean[n];
        boolean[] delivered = new boolean[n];
        List<Visit> visits = new ArrayList<>();
        double[] state = {0, tasks.initialBattery() * capacityJ, 0, 0, 0}; // time, energy, distance, energyUsed, loadedDistance
        int[] position = {DEPOT};
        int load = 0;
        int charges = 0;
        double tardiness = 0;
        int lateTasks = 0;
        double reserve = tasks.reserve() * capacityJ;
        visits.add(new Visit("depot", "start", -1, points[DEPOT].x(), points[DEPOT].y(), 0, 0, 0,
                state[1] / capacityJ, 0));
        while (!pending.isEmpty()) {
            int chosen = -1;
            for (int i = 0; i < pending.size() && chosen < 0; i++) {
                int s = pending.get(i);
                int k = s / 2;
                boolean feasible = s % 2 == 0 ? load < tasks.capacity() && prerequisitesDone(k, delivered) : picked[k];
                if (feasible) {
                    chosen = i;
                }
            }
            if (chosen < 0) {
                throw new IllegalStateException("decoder deadlock (should be impossible for an acyclic task set)");
            }
            int s = pending.remove(chosen);
            int k = s / 2;
            int target = 2 + s;
            double need = legEnergy(distance[position[0]][target]) + serviceEnergy()
                    + legEnergy(distance[target][CHARGER]) + reserve;
            if (state[1] < need && position[0] != CHARGER) {
                charges++;
                travel(position, CHARGER, state, load);
                double arrival = state[0];
                state[0] += (capacityJ - state[1]) / tasks.chargeRate();
                state[1] = capacityJ;
                visits.add(new Visit("charger", "charge", -1, points[CHARGER].x(), points[CHARGER].y(), arrival, state[0],
                        load, 1.0, 0));
            }
            travel(position, target, state, load);
            double arrival = state[0];
            state[0] += tasks.serviceTime();
            state[1] -= serviceEnergy();
            state[3] += serviceEnergy();
            double lateness = 0;
            if (s % 2 == 0) {
                picked[k] = true;
                load++;
            } else {
                delivered[k] = true;
                load--;
                double deadline = tasks.tasks().get(k).deadline();
                if (deadline > 0 && state[0] > deadline) {
                    lateness = state[0] - deadline;
                    tardiness += lateness;
                    lateTasks++;
                }
            }
            visits.add(new Visit(labels[target], s % 2 == 0 ? "pickup" : "delivery", k, points[target].x(),
                    points[target].y(), arrival, state[0], load, state[1] / capacityJ, lateness));
        }
        if (state[1] < legEnergy(distance[position[0]][DEPOT]) + reserve && position[0] != CHARGER) {
            charges++;
            travel(position, CHARGER, state, load);
            double arrival = state[0];
            state[0] += (capacityJ - state[1]) / tasks.chargeRate();
            state[1] = capacityJ;
            visits.add(new Visit("charger", "charge", -1, points[CHARGER].x(), points[CHARGER].y(), arrival, state[0], 0, 1.0, 0));
        }
        travel(position, DEPOT, state, load);
        visits.add(new Visit("depot", "end", -1, points[DEPOT].x(), points[DEPOT].y(), state[0], state[0], 0,
                state[1] / capacityJ, 0));
        return new Schedule(visits, state[0], tardiness, state[2], state[3], charges, lateTasks, state[4]);
    }

    private boolean prerequisitesDone(int k, boolean[] delivered) {
        for (int d : after[k]) {
            if (!delivered[d]) {
                return false;
            }
        }
        return true;
    }

    private void travel(int[] position, int target, double[] state, int load) {
        double d = distance[position[0]][target];
        double e = legEnergy(d);
        state[0] += d / tasks.speed();
        state[1] -= e;
        state[2] += d;
        state[3] += e;
        if (load > 0) {
            state[4] += d;
        }
        position[0] = target;
    }

    /** Conventional baseline: always go to the nearest stop that is feasible now. */
    public Permutation greedyOrder() {
        int n = tasks.tasks().size();
        boolean[] used = new boolean[2 * n];
        boolean[] picked = new boolean[n];
        boolean[] delivered = new boolean[n];
        int[] order = new int[2 * n];
        int position = DEPOT;
        int load = 0;
        for (int step = 0; step < 2 * n; step++) {
            int best = -1;
            for (int s = 0; s < 2 * n; s++) {
                if (used[s]) {
                    continue;
                }
                int k = s / 2;
                boolean feasible = s % 2 == 0 ? load < tasks.capacity() && prerequisitesDone(k, delivered) : picked[k];
                if (feasible && (best < 0 || distance[position][2 + s] < distance[position][2 + best])) {
                    best = s;
                }
            }
            used[best] = true;
            order[step] = best;
            position = 2 + best;
            if (best % 2 == 0) {
                picked[best / 2] = true;
                load++;
            } else {
                delivered[best / 2] = true;
                load--;
            }
        }
        return new Permutation(order);
    }

    @Override
    public Evaluation evaluate(Permutation priority) {
        Schedule s = decode(priority);
        double fitness = switch (objective) {
            case "makespan" -> s.makespan();
            case "tardiness" -> s.tardiness();
            case "distance" -> s.distance();
            case "energy" -> s.energy();
            default -> s.makespan() + weightTardiness * s.tardiness() + weightDistance * s.distance()
                    + weightEnergy * s.energy();
        };
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("makespan", s.makespan());
        metrics.put("tardiness", s.tardiness());
        metrics.put("lateTasks", (double) s.lateTasks());
        metrics.put("distance", s.distance());
        metrics.put("energy", s.energy());
        metrics.put("charges", (double) s.charges());
        metrics.put("utilization", s.distance() > 0 ? s.loadedDistance() / s.distance() : 0);
        double[] objectives = {s.makespan(), s.tardiness(), s.energy()};
        double[] reference = referencePoint();
        double[] behaviour = new double[objectives.length];
        for (int i = 0; i < objectives.length; i++) {
            behaviour[i] = objectives[i] / reference[i];
        }
        return new Evaluation(fitness, hardDeadlines ? s.tardiness() : 0, metrics, objectives, behaviour);
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("makespan", "tardiness", "energy");
    }

    @Override
    public double[] referencePoint() {
        return reference.clone();
    }

    /** All stops with their labels, for drawing. */
    public List<Map<String, Object>> stops() {
        List<Map<String, Object>> out = new ArrayList<>();
        for (int i = 0; i < points.length; i++) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("label", labels[i]);
            m.put("kind", i == DEPOT ? "depot" : i == CHARGER ? "charger" : (i % 2 == 0 ? "pickup" : "delivery"));
            m.put("x", points[i].x());
            m.put("y", points[i].y());
            if (i >= 2) {
                TaskSet.Task t = tasks.tasks().get((i - 2) / 2);
                m.put("task", t.id());
                m.put("deadline", t.deadline());
                m.put("after", t.after());
            }
            out.add(m);
        }
        return out;
    }

    @Override
    public Object view(Permutation priority) {
        Map<String, Object> view = new LinkedHashMap<>();
        Schedule s = decode(priority);
        view.put("schedule", s.visits());
        view.put("stops", stops());
        view.put("genome", priority.toArray());
        return view;
    }

    /**
     * Executes the decoded mission in the simulator, leg by leg along A* routes with the waypoint
     * follower, and stitches the legs into one trajectory. Service and charging times appear as
     * pauses. Reports simulated against modelled travel time.
     */
    public Map<String, Object> replay(Permutation priority) {
        Schedule schedule = decode(priority);
        OccupancyGrid grid = new OccupancyGrid(world, 0.1, robot.radius() + 0.1);
        Trajectory trajectory = new Trajectory(0.1, false);
        List<double[]> route = new ArrayList<>();
        double clock = 0;
        double travelSimulated = 0;
        int failedLegs = 0;
        Pose pose = new Pose(points[DEPOT].x(), points[DEPOT].y(), 0);
        route.add(new double[] {pose.x(), pose.y()});
        List<Visit> visits = schedule.visits();
        for (int i = 1; i < visits.size(); i++) {
            Visit from = visits.get(i - 1);
            Visit to = visits.get(i);
            Vec2 goal = new Vec2(to.x(), to.y());
            if (pose.position().distanceTo(goal) > 0.3) {
                GridPlanner.Plan plan = GridPlanner.astar(grid, pose.position(), goal);
                List<Vec2> waypoints = plan.found() ? plan.waypoints() : List.of(goal);
                for (Vec2 w : waypoints) {
                    route.add(new double[] {w.x(), w.y()});
                }
                double heading = Math.atan2(waypoints.get(0).y() - pose.y(), waypoints.get(0).x() - pose.x());
                Pose start = new Pose(pose.x(), pose.y(), heading);
                double estimate = distance(from, to) / 0.3 + 30;
                EpisodeConfig cfg = EpisodeConfig.defaults()
                        .withMaxSteps((int) Math.ceil(estimate / 0.05))
                        .withAutoComplete(true)
                        .withBattery(Math.max(0.01, from.battery()))
                        .recording(2, false);
                ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), goal, 0.3, waypoints,
                        Params.of("cruiseSpeed", tasks.speed()));
                EpisodeResult r = Simulator.run(world, robot, start, goal, 0.3, ControllerRegistry.create("waypoint-follower"),
                        setup, cfg);
                trajectory.append(r.trajectory(), clock);
                clock += r.time();
                travelSimulated += r.time();
                if (!r.success()) {
                    failedLegs++;
                }
                pose = r.success() ? r.finalPose() : new Pose(goal.x(), goal.y(), r.finalPose().theta());
            }
            clock += to.departure() - to.arrival();
        }
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("outcome", failedLegs == 0 ? "REACHED_GOAL" : "LEG_FAILED");
        Map<String, Double> metrics = new LinkedHashMap<>(evaluate(priority).metrics());
        metrics.put("simulatedMissionTime", clock);
        metrics.put("simulatedTravelTime", travelSimulated);
        metrics.put("modelledTravelTime", schedule.distance() / tasks.speed());
        metrics.put("failedLegs", (double) failedLegs);
        out.put("metrics", metrics);
        out.put("trajectory", trajectory);
        out.put("route", route);
        out.put("schedule", schedule.visits());
        out.put("stops", stops());
        return out;
    }

    private double distance(Visit a, Visit b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y()) * 1.5;
    }

    /** The greedy baseline and its evaluation, reported like the A* reference for navigation. */
    public Map<String, Object> reference() {
        Permutation greedy = greedyOrder();
        Map<String, Object> g = new LinkedHashMap<>();
        Evaluation e = evaluate(greedy);
        g.put("fitness", e.fitness());
        g.put("metrics", e.metrics());
        g.put("objectives", e.objectives());
        g.put("genome", greedy.toArray());
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("greedy", g);
        out.put("referencePoint", reference);
        out.put("note", "Greedy = always serve the nearest stop that is feasible now (a conventional dispatching "
                + "heuristic). The hypervolume reference point is 1.5 x its objectives (+60 s tardiness).");
        return out;
    }
}
