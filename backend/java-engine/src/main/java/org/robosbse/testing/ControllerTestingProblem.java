package org.robosbse.testing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.planning.DistanceField;
import org.robosbse.planning.OccupancyGrid;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;
import org.robosbse.testing.coverage.Branch;
import org.robosbse.testing.coverage.BranchFitness;
import org.robosbse.testing.coverage.CoverageAccumulator;
import org.robosbse.testing.coverage.CoverageTrace;
import org.robosbse.testing.coverage.Instrumented;

/**
 * Search-based testing of robot-controller software: search for test scenarios that make the
 * controller violate a declared property.
 *
 * <p><b>Representation.</b> A bounded real vector:
 * <pre>
 *   0 start.x  1 start.y  2 start.theta  3 goal.x  4 goal.y
 *   5 sensor noise sigma   6 scan dropout probability   7 initial battery   8 speed limit
 *   9 free-distance setting of the controller (controller configuration)
 *   10 + 5k .. 14 + 5k: obstacle slot k = (x, y, size, shape, active)
 * </pre>
 * An obstacle slot is a circle (shape &lt; 0.5) or a square, present only when active &ge; 0.5.
 * Decoding drops slot obstacles that would overlap the start or the goal region, so every decoded
 * scenario starts collision-free. Scenarios whose start or goal overlaps the base arena, or whose
 * start and goal are less than 2 m apart, are invalid: their fitness is {@code 2 + violation}, worse
 * than any valid scenario (valid fitness lies in [-1, 1]).
 *
 * <p><b>Fitness (minimised) — falsification by robustness.</b> The scenario is simulated and the
 * {@link Oracle} computes a normalised robustness for each selected property. With
 * {@code fitness = min} the fitness is the minimum robustness over the selected properties (the
 * robustness of their conjunction); a negative value means at least one property is violated. With
 * {@code fitness = <property id>} only that property's robustness guides the search. This matters
 * because properties live on different scales: under min-aggregation a correctly reported
 * completion (robustness about 0.2) masks the clearance gradient of the collision property once the
 * clearance exceeds 0.05 m. In both modes a violation of <em>any</em> selected property counts as a
 * discovered failure ({@link #isTarget}). Sensor noise uses a fixed seed for every candidate (common random
 * numbers), so robustness changes smoothly with the noise and dropout genes.
 *
 * <p><b>White-box guidance.</b> With {@code fitness = branch:<decision>:<true|false>} the search is
 * first guided to an outcome of an instrumented decision of the controller, then to a violation:
 * <pre>
 *   fitness = 1 + approachLevel + d / (d + 1)      while the outcome is not covered (>= 1)
 *   fitness = rho / (1 + |rho|)                     once it is covered (in (-1, 1); negative = violation)
 * </pre>
 * where rho is the robustness as above. Invalid scenarios then get {@code 100 + violation}. With
 * {@code target = branch} covering the outcome itself counts as the target (used by coverage
 * campaigns). Every episode of an instrumented controller also feeds a per-problem
 * {@link CoverageAccumulator}: the branch coverage achieved by all scenarios the search generated.
 */
public final class ControllerTestingProblem implements Problem<RealVector> {

    public static final String TYPE = "controller-testing";
    public static final int HEADER_GENES = 10;
    public static final int GENES_PER_OBSTACLE = 5;

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.text("controller", "reactive-bug1-dropout", "Controller under test (reactive family)."),
            ParamSpec.text("properties", "safety",
                    "Properties to falsify: 'safety', 'all' or a comma list of " + Property.ids() + "."),
            ParamSpec.text("fitness", "min",
                    "'min' = minimum robustness over the selected properties, one property id to guide the search, "
                            + "or 'branch:<decision>:<true|false>' to reach a decision outcome of the controller first."),
            ParamSpec.choice("target", List.of("violation", "branch"),
                    "What counts as a target hit: a property violation, or covering the 'branch:' fitness outcome."),
            ParamSpec.integer("obstacleSlots", 4, 0, 20, "Number of obstacle slots in the scenario genome."),
            ParamSpec.integer("maxSteps", 1200, 10, 1_000_000, "Episode step limit (dt = 0.05 s)."),
            ParamSpec.integer("simulationSeed", 1, 0, Integer.MAX_VALUE, "Seed of the sensor-noise stream."),
            ParamSpec.real("maxNoise", 0.03, 0, 0.5, "Upper bound of the sensor-noise gene (m)."),
            ParamSpec.real("maxDropout", 0.6, 0, 1, "Upper bound of the scan-dropout gene."),
            ParamSpec.real("minBattery", 0.05, 0.001, 1, "Lower bound of the initial-battery gene."),
            ParamSpec.real("minSpeed", 0.3, 0.05, 5, "Lower bound of the speed-limit gene (m/s)."),
            ParamSpec.real("maxSpeed", 1.0, 0.05, 5, "Upper bound of the speed-limit gene (m/s)."),
            ParamSpec.real("minFreeDistance", 0.4, 0.1, 10, "Lower bound of the free-distance configuration gene (m)."),
            ParamSpec.real("maxFreeDistance", 1.5, 0.1, 10, "Upper bound of the free-distance configuration gene (m)."),
            ParamSpec.real("minObstacleSize", 0.2, 0.05, 5, "Smallest obstacle radius / half-size (m)."),
            ParamSpec.real("maxObstacleSize", 0.8, 0.05, 5, "Largest obstacle radius / half-size (m)."),
            ParamSpec.real("minClearance", 0.03, 0, 2, "Required clearance for the 'clearance' property (m)."));

    private final WorldMap arena;
    private final World baseWorld;
    private final RobotSpec robot;
    private final String controllerId;
    private final Params controllerParams;
    private final Set<Property> properties;
    private final Property guide;
    private final int slots;
    private final int maxSteps;
    private final long simulationSeed;
    private final double minClearance;
    private final Params params;
    private final RealVectorSpace space;
    private final List<Branch> branches;
    private final String instrumentation;
    private final CoverageAccumulator coverage;
    private final String targetDecision;
    private final boolean targetOutcome;
    private final boolean branchIsTarget;

    public ControllerTestingProblem(WorldMap arena, Params params, Params controllerParams, RobotSpec robot) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid testing parameters: " + String.join("; ", errors));
        }
        this.arena = arena;
        this.baseWorld = new World(arena);
        this.robot = robot;
        this.params = params;
        this.controllerId = params.getString("controller", "reactive-bug1-dropout");
        if (!"reactive".equals(ControllerRegistry.get(controllerId).kind())) {
            throw new IllegalArgumentException("controller testing needs a reactive controller, got " + controllerId);
        }
        this.controllerParams = controllerParams;
        this.properties = Property.parseSet(params.getString("properties", "safety"));
        Controller probe = ControllerRegistry.create(controllerId);
        this.branches = probe instanceof Instrumented inst ? inst.branches() : List.of();
        this.instrumentation = probe instanceof Instrumented inst ? inst.instrumentation() : null;
        this.coverage = branches.isEmpty() ? null : new CoverageAccumulator(branches);
        String fitness = params.getString("fitness", "min");
        if (fitness.startsWith("branch:")) {
            String[] parts = fitness.split(":");
            if (parts.length != 3 || !List.of("true", "false").contains(parts[2])) {
                throw new IllegalArgumentException("branch fitness must look like 'branch:<decision>:<true|false>', got " + fitness);
            }
            if (branches.isEmpty()) {
                throw new IllegalArgumentException("controller " + controllerId + " has no coverage instrumentation");
            }
            if (branches.stream().noneMatch(b -> b.id().equals(parts[1]))) {
                throw new IllegalArgumentException("controller " + controllerId + " has no decision '" + parts[1]
                        + "'; instrumented: " + branches.stream().map(Branch::id).toList());
            }
            this.targetDecision = parts[1];
            this.targetOutcome = Boolean.parseBoolean(parts[2]);
            this.guide = null;
        } else {
            this.targetDecision = null;
            this.targetOutcome = false;
            this.guide = fitness.equals("min") ? null : Property.byId(fitness);
            if (guide != null && !properties.contains(guide)) {
                throw new IllegalArgumentException("fitness property '" + fitness + "' must be one of the selected properties");
            }
        }
        this.branchIsTarget = params.getString("target", "violation").equals("branch");
        if (branchIsTarget && targetDecision == null) {
            throw new IllegalArgumentException("target = branch needs fitness = branch:<decision>:<true|false>");
        }
        this.slots = params.getInt("obstacleSlots", 4);
        this.maxSteps = params.getInt("maxSteps", 1200);
        this.simulationSeed = params.getLong("simulationSeed", 1);
        this.minClearance = params.getDouble("minClearance", 0.03);

        int n = HEADER_GENES + GENES_PER_OBSTACLE * slots;
        double[] lo = new double[n];
        double[] hi = new double[n];
        List<String> names = new ArrayList<>();
        double w = arena.width();
        double h = arena.height();
        set(lo, hi, names, 0, "start.x", 0.5, w - 0.5);
        set(lo, hi, names, 1, "start.y", 0.5, h - 0.5);
        set(lo, hi, names, 2, "start.theta", -Math.PI, Math.PI);
        set(lo, hi, names, 3, "goal.x", 0.5, w - 0.5);
        set(lo, hi, names, 4, "goal.y", 0.5, h - 0.5);
        set(lo, hi, names, 5, "sensorNoise", 0, params.getDouble("maxNoise", 0.03));
        set(lo, hi, names, 6, "scanDropout", 0, params.getDouble("maxDropout", 0.6));
        set(lo, hi, names, 7, "battery", params.getDouble("minBattery", 0.05), 1.0);
        set(lo, hi, names, 8, "speedLimit", params.getDouble("minSpeed", 0.3), params.getDouble("maxSpeed", 1.0));
        set(lo, hi, names, 9, "freeDistance", params.getDouble("minFreeDistance", 0.4),
                params.getDouble("maxFreeDistance", 1.5));
        for (int k = 0; k < slots; k++) {
            int b = HEADER_GENES + GENES_PER_OBSTACLE * k;
            set(lo, hi, names, b, "obs" + k + ".x", 0, w);
            set(lo, hi, names, b + 1, "obs" + k + ".y", 0, h);
            set(lo, hi, names, b + 2, "obs" + k + ".size", params.getDouble("minObstacleSize", 0.2),
                    params.getDouble("maxObstacleSize", 0.8));
            set(lo, hi, names, b + 3, "obs" + k + ".shape", 0, 1);
            set(lo, hi, names, b + 4, "obs" + k + ".active", 0, 1);
        }
        this.space = new RealVectorSpace(lo, hi, names, 1);
    }

    private static void set(double[] lo, double[] hi, List<String> names, int i, String name, double a, double b) {
        lo[i] = a;
        hi[i] = b;
        names.add(name);
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    public WorldMap arena() {
        return arena;
    }

    public RobotSpec robot() {
        return robot;
    }

    public String controllerId() {
        return controllerId;
    }

    public Params controllerParams() {
        return controllerParams;
    }

    public Set<Property> properties() {
        return properties;
    }

    public int maxSteps() {
        return maxSteps;
    }

    public long simulationSeed() {
        return simulationSeed;
    }

    public double minClearance() {
        return minClearance;
    }

    public Params params() {
        return params;
    }

    /** Instrumented decisions of the controller under test (empty if it has no instrumentation). */
    public List<Branch> branches() {
        return branches;
    }

    /** What was instrumented, or null when coverage is unavailable for this controller. */
    public String instrumentation() {
        return instrumentation;
    }

    /** Branch coverage of every scenario evaluated so far, or null when coverage is unavailable. */
    public CoverageAccumulator coverage() {
        return coverage;
    }

    /** Genotype-to-phenotype mapping (includes the start/goal keep-out repair of slot obstacles). */
    public Scenario decode(RealVector g) {
        Pose start = new Pose(g.get(0), g.get(1), g.get(2));
        Vec2 goal = new Vec2(g.get(3), g.get(4));
        double keepOut = robot.radius() + 0.15;
        List<Shape> obstacles = new ArrayList<>();
        for (int k = 0; k < slots; k++) {
            int b = HEADER_GENES + GENES_PER_OBSTACLE * k;
            if (g.get(b + 4) < 0.5) {
                continue;
            }
            double x = g.get(b);
            double y = g.get(b + 1);
            double size = g.get(b + 2);
            Shape shape = g.get(b + 3) < 0.5
                    ? new Circle(x, y, size)
                    : new Rect(x - size, y - size, 2 * size, 2 * size);
            if (shape.signedDistance(start.x(), start.y()) < keepOut
                    || shape.signedDistance(goal.x(), goal.y()) < keepOut) {
                continue;
            }
            obstacles.add(shape);
        }
        return new Scenario(start, goal, g.get(5), g.get(6), g.get(7), g.get(8), g.get(9), obstacles);
    }

    /**
     * Whether every obstacle keeps the start/goal keep-out distance that {@link #decode} enforces.
     * Scenarios edited outside the genome (minimisation, the UI) must satisfy it, otherwise the robot
     * could start in contact with an obstacle.
     */
    public boolean obstaclesClearOfEndpoints(Scenario s) {
        double keepOut = robot.radius() + 0.15;
        for (Shape shape : s.obstacles()) {
            if (shape.signedDistance(s.start().x(), s.start().y()) < keepOut
                    || shape.signedDistance(s.goal().x(), s.goal().y()) < keepOut) {
                return false;
            }
        }
        return true;
    }

    /** Amount by which a scenario violates the validity constraints (0 = valid). */
    public double invalidity(Scenario s) {
        double r = robot.radius();
        double startClear = baseWorld.distance(s.start().x(), s.start().y()) - r;
        double goalClear = baseWorld.distance(s.goal().x(), s.goal().y()) - r;
        double separation = s.start().position().distanceTo(s.goal());
        return Math.max(0, 0.1 - startClear) + Math.max(0, 0.1 - goalClear) + Math.max(0, 2.0 - separation);
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluateScenario(decode(genome), genome.toArray());
    }

    public Evaluation evaluateScenario(Scenario scenario) {
        return evaluateScenario(scenario, null);
    }

    private Evaluation evaluateScenario(Scenario scenario, double[] genome) {
        double invalid = invalidity(scenario);
        if (invalid > 0) {
            return new Evaluation((targetDecision == null ? 2 : 100) + invalid, invalid, Map.of("valid", 0.0));
        }
        Execution ex = execute(scenario, controllerId, false);
        double fitness = Double.POSITIVE_INFINITY;
        Map<String, Double> metrics = new LinkedHashMap<>();
        metrics.put("valid", 1.0);
        metrics.put("outcome", (double) ex.result().outcome().ordinal());
        metrics.put("time", ex.result().time());
        metrics.put("minClearance", ex.result().minObstacleClearance());
        metrics.put("maxCommandedSpeed", ex.result().maxCommandedSpeed());
        metrics.put("obstacles", (double) scenario.obstacles().size());
        int violated = 0;
        for (Verdict v : ex.verdicts()) {
            boolean guiding = guide == null || guide.id().equals(v.property());
            if (v.applicable() && guiding) {
                fitness = Math.min(fitness, v.robustness());
            }
            if (v.violated()) {
                violated++;
            }
            metrics.put("rho." + v.property(), v.robustness());
        }
        metrics.put("violations", (double) violated);
        double value = Double.isFinite(fitness) ? fitness : 1.0;
        CoverageTrace trace = ex.coverage();
        if (trace != null) {
            metrics.put("coveredOutcomes", (double) trace.coveredOutcomes());
            if (coverage != null && genome != null) {
                coverage.add(trace, genome);
            }
        }
        if (targetDecision != null) {
            BranchFitness.Value b = BranchFitness.of(trace, targetDecision, targetOutcome);
            metrics.put("approachLevel", (double) b.approachLevel());
            metrics.put("branchDistance", Double.isFinite(b.distance()) ? b.distance() : -1);
            metrics.put("targetCovered", b.covered() ? 1.0 : 0.0);
            value = b.covered() ? value / (1 + Math.abs(value)) : 1 + b.fitness();
        }
        return new Evaluation(value, 0, metrics, null, behaviour(ex.result()));
    }

    /**
     * Behaviour of the controller under test: its path (final position and positions at 75/50/25% of
     * the episode, normalised by the arena size), the closest approach to an obstacle (clamped to
     * [0, 1] m) and the episode duration as a fraction of the step limit.
     */
    private double[] behaviour(EpisodeResult r) {
        double[] path = r.behaviour(arena.width(), arena.height());
        double[] out = Arrays.copyOf(path, path.length + 2);
        out[path.length] = Math.max(0, Math.min(1, r.minObstacleClearance()));
        out[path.length + 1] = (double) r.steps() / maxSteps;
        return out;
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        if (branchIsTarget) {
            return evaluation.metric("targetCovered", 0) > 0;
        }
        return evaluation.feasible() && evaluation.metric("violations", 0) > 0;
    }

    /**
     * Result of running one controller on one scenario.
     *
     * @param coverage coverage trace of the episode, or null if the controller is not instrumented
     */
    public record Execution(Scenario scenario, String controller, EpisodeResult result, List<Verdict> verdicts,
                            CoverageTrace coverage) {

        /** Labels of the decision outcomes this episode covered (empty without instrumentation). */
        public List<String> coveredOutcomes() {
            List<String> out = new ArrayList<>();
            if (coverage != null) {
                for (int i = 0; i < coverage.branches().size(); i++) {
                    for (boolean outcome : new boolean[] {true, false}) {
                        if (coverage.covered(i, outcome)) {
                            out.add(Branch.outcome(coverage.branches().get(i).id(), outcome));
                        }
                    }
                }
            }
            return out;
        }

        public List<String> violated() {
            List<String> ids = new ArrayList<>();
            for (Verdict v : verdicts) {
                if (v.violated()) {
                    ids.add(v.property());
                }
            }
            return ids;
        }
    }

    /** Runs {@code controller} on the scenario (optionally recording a replay) and judges it. */
    public Execution execute(Scenario scenario, String controller, boolean record) {
        return execute(scenario, controller, ControllerRegistry.create(controller), record);
    }

    /**
     * Runs a given controller instance (e.g. a compiled repair candidate) on the scenario and judges it;
     * {@code label} names it in the execution record.
     */
    public Execution execute(Scenario scenario, String label, Controller instance, boolean record) {
        World world = baseWorld.withExtraObstacles(scenario.obstacles());
        EpisodeConfig cfg = EpisodeConfig.defaults()
                .withMaxSteps(maxSteps)
                .withAutoComplete(false)
                .withSensing(scenario.sensorNoise(), scenario.scanDropout(), simulationSeed)
                .withBattery(scenario.battery());
        if (record) {
            cfg = cfg.recording(1, true);
        }
        Params cp = controllerParams.with("speedLimit", scenario.speedLimit())
                .with("freeDistance", scenario.freeDistance());
        ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), scenario.goal(), arena.goalTolerance(),
                List.of(scenario.goal()), cp);
        CoverageTrace trace = null;
        if (instance instanceof Instrumented inst) {
            trace = new CoverageTrace(inst.branches());
            inst.attach(trace);
        }
        EpisodeResult result = Simulator.run(world, robot, scenario.start(), scenario.goal(), arena.goalTolerance(),
                instance, setup, cfg);
        Oracle.LivenessContext liveness = properties.contains(Property.LIVENESS) || record
                ? livenessContext(world, scenario, result)
                : null;
        Set<Property> judged = record ? java.util.EnumSet.allOf(Property.class) : properties;
        List<Verdict> verdicts = Oracle.judge(result, robot, arena.goalTolerance(), minClearance, judged, liveness);
        if (record) {
            // Report every property, but only the selected ones count as violations of this test.
            List<Verdict> filtered = new ArrayList<>();
            for (Verdict v : verdicts) {
                boolean selected = properties.contains(Property.byId(v.property()));
                filtered.add(selected ? v : new Verdict(v.property(), v.applicable(), false, v.robustness(),
                        v.detail() + " (not selected)"));
            }
            verdicts = filtered;
        }
        return new Execution(scenario, label, result, verdicts, trace);
    }

    private Oracle.LivenessContext livenessContext(World world, Scenario s, EpisodeResult result) {
        OccupancyGrid grid = new OccupancyGrid(world, 0.1, robot.radius() + 0.02);
        DistanceField field = new DistanceField(grid, s.goal());
        boolean reachable = field.reachable(s.start().x(), s.start().y());
        double initial = reachable ? field.distanceFrom(s.start().x(), s.start().y()) : Double.POSITIVE_INFINITY;
        double needed = reachable
                ? 1.3 * (initial / s.speedLimit()) * robot.power(s.speedLimit(), 0, 0)
                : Double.POSITIVE_INFINITY;
        boolean energy = s.battery() * robot.batteryCapacity() >= needed;
        double fin = field.distanceFrom(result.finalPose().x(), result.finalPose().y());
        return new Oracle.LivenessContext(reachable, energy, initial, fin);
    }

    @Override
    public Object view(RealVector genome) {
        Map<String, Object> view = new LinkedHashMap<>();
        view.put("scenario", decode(genome));
        view.put("genome", genome.toArray());
        return view;
    }
}
