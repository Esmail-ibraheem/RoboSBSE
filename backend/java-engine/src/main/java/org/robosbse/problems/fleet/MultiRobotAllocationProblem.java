package org.robosbse.problems.fleet;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
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
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.Trajectory;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Multi-robot task allocation: which robot does which transport task, and in which order.
 *
 * <p><b>Encoding: random keys</b>, {@code 2M} genes for M tasks: an assignment gene in [0, R) (the robot is
 * its integer part) and a priority in [0, 1] (each robot serves its tasks in increasing priority). Every
 * real-vector algorithm applies.
 *
 * <p><b>Cost model.</b> A robot drives depot, pickup, delivery, pickup, delivery, ... (and back to its
 * depot when {@code returnHome}); legs use geodesic distances on the inflated occupancy grid at the
 * planning speed, each pickup and delivery takes {@code serviceTime}. Objectives: (makespan = time of the
 * last robot to finish, total distance). Fitness = makespan + {@code weightDistance} x total distance.
 * Reference: greedy list scheduling (repeatedly the task and robot that finish earliest).
 *
 * <p><b>Validation by simulation.</b> Replays execute every robot's plan with the waypoint follower along
 * A* routes, robot by robot. Robots are simulated independently, so they do not avoid each other;
 * afterwards every pair of trajectories is compared at equal times and each time two robots come closer
 * than two radii plus 0.1 m is reported as a <em>conflict</em>. Conflicts are measured, not resolved.
 */
public final class MultiRobotAllocationProblem implements Problem<RealVector> {

    public static final String TYPE = "multi-robot-allocation";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("fleet", "deliveries-12", "Fleet instance: a built-in name or a robosbse.fleet/1 file."),
            ParamSpec.real("weightDistance", 0.1, 0, 1000, "Weight of the total distance (s per metre)."));

    private final Fleet fleet;
    private final WorldMap map;
    private final World world;
    private final RobotSpec robot;
    private final RealVectorSpace space;
    private final int robots;
    private final int tasks;
    private final double weightDistance;
    /** Points: robots 0..R-1, then pickup of task t at R + 2t, delivery at R + 2t + 1. */
    private final Vec2[] points;
    private final double[][] distance;
    private final double[] reference;

    public MultiRobotAllocationProblem(Fleet fleet, WorldMap map, Params params, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid multi-robot parameters: " + String.join("; ", errors));
        }
        this.fleet = fleet;
        this.map = map;
        this.world = new World(map);
        this.robot = robot;
        this.robots = fleet.robots().size();
        this.tasks = fleet.tasks().size();
        this.weightDistance = params.getDouble("weightDistance", 0.1);
        double[] lo = new double[2 * tasks];
        double[] hi = new double[2 * tasks];
        List<String> names = new ArrayList<>();
        for (int t = 0; t < tasks; t++) {
            hi[t] = robots - 1e-9;
            names.add("robot of " + fleet.tasks().get(t).label());
        }
        for (int t = 0; t < tasks; t++) {
            hi[tasks + t] = 1;
            names.add("priority of " + fleet.tasks().get(t).label());
        }
        this.space = new RealVectorSpace(lo, hi, names, 1);
        this.points = new Vec2[robots + 2 * tasks];
        for (int r = 0; r < robots; r++) {
            points[r] = fleet.robots().get(r);
        }
        for (int t = 0; t < tasks; t++) {
            points[robots + 2 * t] = fleet.tasks().get(t).pickup();
            points[robots + 2 * t + 1] = fleet.tasks().get(t).delivery();
        }
        OccupancyGrid grid = new OccupancyGrid(world, 0.1, robot.radius() + 0.1);
        this.distance = new double[points.length][points.length];
        for (int b = 0; b < points.length; b++) {
            DistanceField field = new DistanceField(grid, points[b]);
            for (int a = 0; a < points.length; a++) {
                if (a != b && !field.reachable(points[a].x(), points[a].y())) {
                    throw new IllegalArgumentException(points[b] + " is not reachable from " + points[a] + " on " + map.name());
                }
                distance[a][b] = a == b ? 0 : field.distanceFrom(points[a].x(), points[a].y());
            }
        }
        double[] greedy = evaluate(greedy()).objectives();
        this.reference = new double[] {2 * greedy[0], 2 * greedy[1]};
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    public Fleet fleet() {
        return fleet;
    }

    public WorldMap map() {
        return map;
    }

    /** Task lists per robot, in service order. */
    public int[][] decode(double[] g) {
        List<List<Integer>> lists = new ArrayList<>();
        for (int r = 0; r < robots; r++) {
            lists.add(new ArrayList<>());
        }
        Integer[] order = new Integer[tasks];
        for (int t = 0; t < tasks; t++) {
            order[t] = t;
        }
        Arrays.sort(order, Comparator.comparingDouble((Integer t) -> g[tasks + t]).thenComparingInt(t -> t));
        for (int t : order) {
            int r = Math.min(robots - 1, Math.max(0, (int) Math.floor(g[t])));
            lists.get(r).add(t);
        }
        int[][] out = new int[robots][];
        for (int r = 0; r < robots; r++) {
            out[r] = lists.get(r).stream().mapToInt(Integer::intValue).toArray();
        }
        return out;
    }

    /** Route length (m) and duration (s) of one robot's task list. */
    double[] robotCost(int r, int[] list) {
        double length = 0;
        int at = r;
        for (int t : list) {
            length += distance[at][robots + 2 * t] + distance[robots + 2 * t][robots + 2 * t + 1];
            at = robots + 2 * t + 1;
        }
        if (fleet.returnHome() && list.length > 0) {
            length += distance[at][r];
        }
        return new double[] {length, length / fleet.speed() + 2 * list.length * fleet.serviceTime()};
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluate(decode(genome.toArray()));
    }

    public Evaluation evaluate(int[][] plan) {
        double makespan = 0;
        double total = 0;
        double busiest = 0;
        double idlest = Double.POSITIVE_INFINITY;
        Map<String, Double> m = new LinkedHashMap<>();
        for (int r = 0; r < robots; r++) {
            double[] c = robotCost(r, plan[r]);
            makespan = Math.max(makespan, c[1]);
            total += c[0];
            busiest = Math.max(busiest, c[1]);
            idlest = Math.min(idlest, c[1]);
            m.put("time.robot" + r, c[1]);
            m.put("tasks.robot" + r, (double) plan[r].length);
        }
        double fitness = makespan + weightDistance * total;
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("makespan", makespan);
        metrics.put("totalDistance", total);
        metrics.put("imbalance", busiest - idlest);
        metrics.putAll(m);
        double first = plan[0].length / (double) tasks;
        return new Evaluation(fitness, 0, metrics, new double[] {makespan, total},
                new double[] {first, (busiest - idlest) / Math.max(1e-9, busiest)});
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("makespan", "totalDistance");
    }

    @Override
    public double[] referencePoint() {
        return reference.clone();
    }

    /** Greedy list scheduling: repeatedly append the task that some robot can finish earliest. */
    public int[][] greedy() {
        List<List<Integer>> lists = new ArrayList<>();
        double[] clock = new double[robots];
        int[] at = new int[robots];
        for (int r = 0; r < robots; r++) {
            lists.add(new ArrayList<>());
            at[r] = r;
        }
        boolean[] done = new boolean[tasks];
        for (int step = 0; step < tasks; step++) {
            double best = Double.POSITIVE_INFINITY;
            int br = -1;
            int bt = -1;
            for (int t = 0; t < tasks; t++) {
                if (done[t]) {
                    continue;
                }
                for (int r = 0; r < robots; r++) {
                    double leg = distance[at[r]][robots + 2 * t] + distance[robots + 2 * t][robots + 2 * t + 1];
                    double finish = clock[r] + leg / fleet.speed() + 2 * fleet.serviceTime();
                    if (finish < best) {
                        best = finish;
                        br = r;
                        bt = t;
                    }
                }
            }
            done[bt] = true;
            lists.get(br).add(bt);
            clock[br] = best;
            at[br] = robots + 2 * bt + 1;
        }
        int[][] out = new int[robots][];
        for (int r = 0; r < robots; r++) {
            out[r] = lists.get(r).stream().mapToInt(Integer::intValue).toArray();
        }
        return out;
    }

    /** A genome that decodes to the given plan (for replaying the reference). */
    public double[] encode(int[][] plan) {
        double[] g = new double[2 * tasks];
        for (int r = 0; r < robots; r++) {
            for (int i = 0; i < plan[r].length; i++) {
                int t = plan[r][i];
                g[t] = r + 0.5;
                g[tasks + t] = (i + 0.5) / (plan[r].length + 1.0);
            }
        }
        return g;
    }

    public Map<String, Object> reference() {
        int[][] g = greedy();
        Evaluation e = evaluate(g);
        Map<String, Object> greedy = new LinkedHashMap<>();
        greedy.put("fitness", e.fitness());
        greedy.put("metrics", e.metrics());
        greedy.put("plan", describe(g));
        greedy.put("genome", encode(g));
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("greedy", greedy);
        out.put("note", "Greedy list scheduling: repeatedly the task and robot that finish earliest.");
        return out;
    }

    List<List<String>> describe(int[][] plan) {
        List<List<String>> out = new ArrayList<>();
        for (int[] list : plan) {
            List<String> labels = new ArrayList<>();
            for (int t : list) {
                labels.add(fleet.tasks().get(t).label());
            }
            out.add(labels);
        }
        return out;
    }

    @Override
    public Object view(RealVector genome) {
        Map<String, Object> v = new LinkedHashMap<>();
        v.put("plan", describe(decode(genome.toArray())));
        v.put("stops", stops());
        v.put("genome", genome.toArray());
        return v;
    }

    /** Simulated execution of every robot's plan, plus robot-robot conflicts found afterwards. */
    public Map<String, Object> replay(double[] genome) {
        int[][] plan = decode(genome);
        OccupancyGrid grid = new OccupancyGrid(world, 0.1, robot.radius() + 0.1);
        List<Map<String, Object>> robotsOut = new ArrayList<>();
        List<Trajectory> trajectories = new ArrayList<>();
        int failedLegs = 0;
        for (int r = 0; r < robots; r++) {
            Trajectory trajectory = new Trajectory(0.1, false);
            List<double[]> route = new ArrayList<>();
            List<Vec2> stops = new ArrayList<>();
            for (int t : plan[r]) {
                stops.add(points[robots + 2 * t]);
                stops.add(points[robots + 2 * t + 1]);
            }
            if (fleet.returnHome() && plan[r].length > 0) {
                stops.add(points[r]);
            }
            Pose pose = new Pose(points[r].x(), points[r].y(), 0);
            route.add(new double[] {pose.x(), pose.y()});
            trajectory.hold(0, pose.x(), pose.y(), 0);
            double clock = 0;
            for (int i = 0; i < stops.size(); i++) {
                Vec2 goal = stops.get(i);
                if (pose.position().distanceTo(goal) > 0.3) {
                    GridPlanner.Plan p = GridPlanner.astar(grid, pose.position(), goal);
                    List<Vec2> waypoints = p.found() ? p.waypoints() : List.of(goal);
                    for (Vec2 w : waypoints) {
                        route.add(new double[] {w.x(), w.y()});
                    }
                    double heading = Math.atan2(waypoints.get(0).y() - pose.y(), waypoints.get(0).x() - pose.x());
                    double estimate = pose.position().distanceTo(goal) * 3 / 0.3 + 30;
                    EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps((int) Math.ceil(estimate / 0.05))
                            .withAutoComplete(true).recording(2, false);
                    ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), goal, 0.3, waypoints,
                            Params.of("cruiseSpeed", fleet.speed()));
                    EpisodeResult res = Simulator.run(world, robot, new Pose(pose.x(), pose.y(), heading), goal, 0.3,
                            ControllerRegistry.create("waypoint-follower"), setup, cfg);
                    trajectory.append(res.trajectory(), clock);
                    clock += res.time();
                    failedLegs += res.success() ? 0 : 1;
                    pose = res.success() ? res.finalPose() : new Pose(goal.x(), goal.y(), res.finalPose().theta());
                }
                boolean service = !(fleet.returnHome() && i == stops.size() - 1);
                if (service) {
                    clock += fleet.serviceTime();
                    trajectory.hold(clock, pose.x(), pose.y(), pose.theta());
                }
            }
            trajectories.add(trajectory);
            Map<String, Object> ro = new LinkedHashMap<>();
            ro.put("robot", r);
            ro.put("tasks", describe(plan).get(r));
            ro.put("route", route);
            ro.put("simulatedTime", clock);
            ro.put("trajectory", trajectory);
            robotsOut.add(ro);
        }
        List<Map<String, Object>> conflicts = conflicts(trajectories);
        Map<String, Object> out = new LinkedHashMap<>();
        Map<String, Double> metrics = new LinkedHashMap<>(evaluate(plan).metrics());
        double simulatedMakespan = 0;
        for (Trajectory t : trajectories) {
            simulatedMakespan = Math.max(simulatedMakespan, t.endTime());
        }
        metrics.put("simulatedMakespan", simulatedMakespan);
        metrics.put("conflicts", (double) conflicts.size());
        metrics.put("failedLegs", (double) failedLegs);
        out.put("robots", robotsOut);
        out.put("conflicts", conflicts);
        out.put("metrics", metrics);
        out.put("stops", stops());
        out.put("outcome", failedLegs == 0 ? "REACHED_GOAL" : "LEG_FAILED");
        return out;
    }

    /** Times at which two robots came closer than two radii plus 0.1 m (merged into episodes). */
    List<Map<String, Object>> conflicts(List<Trajectory> trajectories) {
        double end = 0;
        for (Trajectory t : trajectories) {
            end = Math.max(end, t.endTime());
        }
        double limit = 2 * robot.radius() + 0.1;
        List<Map<String, Object>> out = new ArrayList<>();
        for (int a = 0; a < trajectories.size(); a++) {
            for (int b = a + 1; b < trajectories.size(); b++) {
                Map<String, Object> open = null;
                double closest = Double.POSITIVE_INFINITY;
                for (int k = 0; k * 0.1 <= end + 1e-9; k++) {
                    double t = k * 0.1;
                    double[] pa = position(trajectories.get(a), t);
                    double[] pb = position(trajectories.get(b), t);
                    double d = Math.hypot(pa[0] - pb[0], pa[1] - pb[1]);
                    if (d < limit) {
                        if (open == null) {
                            open = new LinkedHashMap<>();
                            open.put("robots", new int[] {a, b});
                            open.put("start", t);
                            open.put("x", (pa[0] + pb[0]) / 2);
                            open.put("y", (pa[1] + pb[1]) / 2);
                            closest = d;
                        }
                        closest = Math.min(closest, d);
                        open.put("end", t);
                        open.put("closestDistance", closest);
                    } else if (open != null) {
                        out.add(open);
                        open = null;
                    }
                }
                if (open != null) {
                    out.add(open);
                }
            }
        }
        out.sort(Comparator.comparingDouble(m -> (Double) m.get("start")));
        return out;
    }

    /** Position at time t (held at the last sample after the trajectory ends). */
    static double[] position(Trajectory tr, double t) {
        double[] times = tr.times();
        double[] xs = tr.xs();
        double[] ys = tr.ys();
        int n = tr.size();
        if (n == 0) {
            return new double[] {Double.NaN, Double.NaN};
        }
        if (t <= times[0]) {
            return new double[] {xs[0], ys[0]};
        }
        if (t >= times[n - 1]) {
            return new double[] {xs[n - 1], ys[n - 1]};
        }
        int lo = 0;
        int hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (times[mid] <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        double f = (t - times[lo]) / Math.max(1e-12, times[hi] - times[lo]);
        return new double[] {xs[lo] + f * (xs[hi] - xs[lo]), ys[lo] + f * (ys[hi] - ys[lo])};
    }

    /** Pickups, deliveries and depots for drawing. */
    public List<Map<String, Object>> stops() {
        List<Map<String, Object>> out = new ArrayList<>();
        for (int r = 0; r < robots; r++) {
            out.add(stop("depot", "R" + (r + 1), points[r]));
        }
        for (int t = 0; t < tasks; t++) {
            String label = fleet.tasks().get(t).label();
            out.add(stop("pickup", label + "+", points[robots + 2 * t]));
            out.add(stop("delivery", label + "-", points[robots + 2 * t + 1]));
        }
        return out;
    }

    private static Map<String, Object> stop(String kind, String label, Vec2 p) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("kind", kind);
        m.put("label", label);
        m.put("x", p.x());
        m.put("y", p.y());
        return m;
    }
}
