package org.robosbse.experiments;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;

import org.robosbse.arm.ArmIkProblem;
import org.robosbse.arm.ArmPickPlaceProblem;
import org.robosbse.arm.ArmScene;
import org.robosbse.arm.ArmTrajectoryProblem;
import org.robosbse.core.Candidate;
import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.Permutation;
import org.robosbse.core.representation.PermutationSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geneticprogramming.ControllerSynthesisProblem;
import org.robosbse.geneticprogramming.Program;
import org.robosbse.geneticprogramming.ProgramSpace;
import org.robosbse.improvement.ImprovementProblem;
import org.robosbse.patching.PatchSpace;
import org.robosbse.patching.Subject;
import org.robosbse.repair.RepairProblem;
import org.robosbse.problems.ControllerTuningProblem;
import org.robosbse.problems.NavigationProblem;
import org.robosbse.problems.WarehouseSchedulingProblem;
import org.robosbse.problems.fleet.Fleet;
import org.robosbse.problems.fleet.MultiRobotAllocationProblem;
import org.robosbse.problems.warehouse.TaskLibrary;
import org.robosbse.problems.warehouse.TaskSet;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.FailureRecord;
import org.robosbse.testing.Failures;
import org.robosbse.testing.Scenario;

/** Builds {@link ProblemInstance}s from the {@code problem} section of an experiment configuration. */
public final class Problems {

    /**
     * Static description of a problem type for the capability message.
     *
     * @param representation solution representation (decides which algorithms and operators apply)
     * @param operators      operator names per kind (mutation, crossover, neighbourhood)
     * @param objectives     objective names for multi-objective algorithms (empty = single-objective only)
     */
    public record Descriptor(String id, String name, String summary, String defaultMap, String representation,
                             Map<String, List<String>> operators, List<String> objectives, List<ParamSpec> params) {
    }

    private static Map<String, List<String>> realOperators() {
        Map<String, List<String>> m = new LinkedHashMap<>();
        m.put("mutation", RealVectorSpace.MUTATIONS);
        m.put("crossover", RealVectorSpace.CROSSOVERS);
        m.put("neighbourhood", List.of("default"));
        return m;
    }

    private static Map<String, List<String>> permutationOperators() {
        Map<String, List<String>> m = new LinkedHashMap<>();
        m.put("mutation", PermutationSpace.MUTATIONS);
        m.put("crossover", PermutationSpace.CROSSOVERS);
        m.put("neighbourhood", PermutationSpace.NEIGHBOURHOODS);
        return m;
    }

    private static Map<String, List<String>> programOperators() {
        Map<String, List<String>> m = new LinkedHashMap<>();
        m.put("mutation", ProgramSpace.MUTATIONS);
        m.put("crossover", ProgramSpace.CROSSOVERS);
        m.put("neighbourhood", List.of("default"));
        return m;
    }

    private static Map<String, List<String>> patchOperators() {
        Map<String, List<String>> m = new LinkedHashMap<>();
        m.put("mutation", PatchSpace.MUTATIONS);
        m.put("crossover", PatchSpace.CROSSOVERS);
        m.put("neighbourhood", List.of("default"));
        return m;
    }

    public static final List<Descriptor> DESCRIPTORS = List.of(
            new Descriptor(NavigationProblem.TYPE, "Route optimisation",
                    "Find waypoints that a route-following controller executes collision-free and fast. Fitness "
                            + "comes from simulating the robot; A* and Dijkstra are reported as references. "
                            + "Objectives for multi-objective search: time, energy, collision risk.",
                    "builtin:warehouse", "real-vector", realOperators(), List.of("time", "energy", "risk"),
                    NavigationProblem.PARAMS),
            new Descriptor(ControllerTuningProblem.TYPE, "Controller parameter tuning",
                    "Tune the gains and thresholds of a real controller so that it performs well across several "
                            + "training maps; generalisation is measured on held-out validation maps.",
                    "builtin:obstacle-course", "real-vector", realOperators(), List.of("time", "energy", "risk"),
                    ControllerTuningProblem.PARAMS),
            new Descriptor(WarehouseSchedulingProblem.TYPE, "Warehouse task scheduling",
                    "Order pickups and deliveries under capacity, dependencies, deadlines and battery limits "
                            + "(permutation + feasibility-preserving decoder). Objectives: makespan, tardiness, energy.",
                    "builtin:warehouse", "permutation", permutationOperators(), List.of("makespan", "tardiness", "energy"),
                    WarehouseSchedulingProblem.PARAMS),
            new Descriptor(ControllerTestingProblem.TYPE, "Controller failure discovery",
                    "Search-based testing: search for scenarios (start, goal, obstacles, sensor noise, dropout, "
                            + "battery, controller configuration) that make a controller violate a safety property.",
                    "builtin:testing-arena", "real-vector", realOperators(), List.of(), ControllerTestingProblem.PARAMS),
            new Descriptor(ControllerSynthesisProblem.TYPE, "Controller synthesis (genetic programming)",
                    "Evolve a reactive controller program in a small typed language (sensor tests, comparisons, "
                            + "arithmetic, if-then-else, motion primitives) that reaches the goal on every training map; "
                            + "generalisation is measured on unseen test maps against the hand-written reactive controller.",
                    "builtin:obstacle-course", "program", programOperators(), List.of("meanCost", "size"),
                    ControllerSynthesisProblem.PARAMS),
            new Descriptor(RepairProblem.TYPE, "Automated program repair",
                    "Search for a small source patch (comparison, arithmetic and logical operators, numeric constants, "
                            + "conditions) that makes a faulty Java controller pass its failing tests without breaking the "
                            + "passing ones. Candidates are compiled and run in separate processes with time limits; plausible "
                            + "patches are minimised and validated on held-out tests.",
                    "builtin:testing-arena", "patch", patchOperators(), List.of(), RepairProblem.PARAMS),
            new Descriptor(ImprovementProblem.TYPE, "Genetic improvement",
                    "Search for constrained source edits that make a working controller faster (travel time) or cheaper "
                            + "(counted work per control step) while it keeps passing its correctness suite. Objectives: time "
                            + "ratio, work ratio (relative to the unchanged baseline).",
                    "builtin:testing-arena", "patch", patchOperators(), List.of("timeRatio", "workRatio"),
                    ImprovementProblem.PARAMS),
            new Descriptor(ArmIkProblem.TYPE, "Arm: inverse kinematics",
                    "Joint angles of a planar arm that put the end effector on the target without collisions or "
                            + "joint-limit violations; analytical (two-link) and numerical IK are the references.",
                    "builtin:arm-reach", "real-vector", realOperators(), List.of("distance", "jointMotion", "risk"),
                    ArmIkProblem.PARAMS),
            new Descriptor(ArmTrajectoryProblem.TYPE, "Arm: trajectory optimisation",
                    "Via-point motion from the home pose to the target: collision-free, little joint travel, smooth, "
                            + "energy-efficient. The straight joint-space move to an IK solution is the reference.",
                    "builtin:arm-cluttered", "real-vector", realOperators(), List.of("time", "effort", "roughness"),
                    ArmTrajectoryProblem.PARAMS),
            new Descriptor(ArmPickPlaceProblem.TYPE, "Arm: pick-and-place planning",
                    "Order of the objects and elbow configuration of every pick and place (random-key encoding); "
                            + "greedy planning and exhaustive enumeration are the references.",
                    "builtin:arm-pick-place", "real-vector", realOperators(), List.of("time", "effort", "risk"),
                    ArmPickPlaceProblem.PARAMS),
            new Descriptor(MultiRobotAllocationProblem.TYPE, "Multi-robot task allocation",
                    "Assign transport tasks to a fleet of robots and order each robot's tasks (random-key encoding). "
                            + "Objectives: makespan, total distance. Greedy list scheduling is the reference; replays "
                            + "simulate every robot and report robot-robot conflicts.",
                    "builtin:delivery-center", "real-vector", realOperators(), List.of("makespan", "totalDistance"),
                    MultiRobotAllocationProblem.PARAMS));

    private Problems() {
    }

    public static ProblemInstance<?> create(JsonNode config, Path baseDir) {
        String type = config.path("type").asText(NavigationProblem.TYPE);
        Params params = Params.of(Json.toMap(config.get("params")));
        Params controllerParams = Params.of(Json.toMap(config.get("controllerParams")));
        RobotSpec robot = robot(config.get("robot"));
        return switch (type) {
            case NavigationProblem.TYPE -> new Navigation(new NavigationProblem(
                    MapLibrary.resolve(config.get("map"), baseDir), params, controllerParams, robot));
            case ControllerTestingProblem.TYPE -> new Testing(new ControllerTestingProblem(
                    MapLibrary.resolve(config.get("map"), baseDir), params, controllerParams, robot));
            case ControllerTuningProblem.TYPE -> {
                List<WorldMap> training = maps(params.getString("maps", "obstacle-course,warehouse,empty-field"), baseDir);
                List<WorldMap> validation = maps(params.getString("validationMaps", "maze"), baseDir);
                yield new Tuning(new ControllerTuningProblem(training, validation, params, controllerParams, robot));
            }
            case ControllerSynthesisProblem.TYPE -> new Synthesis(new ControllerSynthesisProblem(
                    maps(params.getString("maps", "empty-field,obstacle-course,generated:scatter:1,generated:scatter:2"), baseDir),
                    maps(params.getString("testMaps", "warehouse,generated:scatter:101,generated:scatter:102,generated:scatter:103"),
                            baseDir),
                    params, robot));
            case RepairProblem.TYPE -> new PatchInstances.Repair(new RepairProblem(
                    Subject.resolve(params.getString("subject", "rb1-stopping-distance"), baseDir), params, robot));
            case ImprovementProblem.TYPE -> new PatchInstances.Improvement(new ImprovementProblem(
                    Subject.resolve(params.getString("subject", "gi-reactive-baseline"), baseDir),
                    maps(params.getString("maps", "obstacle-course,warehouse,empty-field"), baseDir), params, robot));
            case ArmIkProblem.TYPE -> {
                ArmScene scene = ArmScene.resolve(config.get("scene"), baseDir);
                yield new ArmInstances.Ik(scene, new ArmIkProblem(scene, params));
            }
            case ArmTrajectoryProblem.TYPE -> {
                ArmScene scene = ArmScene.resolve(config.hasNonNull("scene") ? config.get("scene")
                        : TextNode.valueOf("arm-cluttered"), baseDir);
                yield new ArmInstances.Trajectory(scene, new ArmTrajectoryProblem(scene, params));
            }
            case ArmPickPlaceProblem.TYPE -> {
                ArmScene scene = ArmScene.resolve(config.hasNonNull("scene") ? config.get("scene")
                        : TextNode.valueOf("arm-pick-place"), baseDir);
                yield new ArmInstances.PickPlace(scene, new ArmPickPlaceProblem(scene, params));
            }
            case WarehouseSchedulingProblem.TYPE -> {
                Object raw = params.raw("tasks");
                JsonNode ref = raw == null ? TextNode.valueOf("builtin:orders-8") : Json.tree(raw);
                TaskSet tasks = TaskLibrary.resolve(ref, baseDir);
                JsonNode mapRef = config.hasNonNull("map") ? config.get("map") : tasks.map();
                WorldMap map = MapLibrary.resolve(mapRef, baseDir);
                Params rest = params.with("tasks", "inline");
                yield new Warehouse(new WarehouseSchedulingProblem(tasks, map, rest, robot));
            }
            case MultiRobotAllocationProblem.TYPE -> {
                Object raw = params.raw("fleet");
                Fleet fleet = Fleet.resolve(raw == null ? null : Json.tree(raw), baseDir);
                JsonNode mapRef = config.hasNonNull("map") ? config.get("map") : fleet.map();
                WorldMap map = MapLibrary.resolve(mapRef, baseDir);
                yield new MultiRobot(new MultiRobotAllocationProblem(fleet, map, params.with("fleet", "inline"), robot));
            }
            default -> throw new IllegalArgumentException("unknown problem type '" + type + "'");
        };
    }

    private static List<WorldMap> maps(String spec, Path baseDir) {
        List<WorldMap> out = new ArrayList<>();
        for (String part : spec.split(",")) {
            if (!part.isBlank()) {
                out.add(MapLibrary.resolve(TextNode.valueOf(part.trim()), baseDir));
            }
        }
        return out;
    }

    /** Robot parameters: the defaults overridden by any fields given in the configuration. */
    public static RobotSpec robot(JsonNode overrides) {
        ObjectNode base = (ObjectNode) Json.tree(RobotSpec.DEFAULT);
        if (overrides != null && overrides.isObject()) {
            overrides.fields().forEachRemaining(e -> base.set(e.getKey(), e.getValue()));
        }
        return Json.convert(base, RobotSpec.class);
    }

    static Map<String, Object> episode(EpisodeResult r) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("outcome", r.outcome().name());
        m.put("metrics", r.metrics());
        m.put("trajectory", r.trajectory());
        return m;
    }

    static RealVector realGenome(JsonNode genome) {
        return new RealVector(Json.convert(genome, double[].class));
    }

    /** Route optimisation. */
    /** Genetic programming of controllers: replays on training and test maps, reactive reference. */
    static final class Synthesis implements ProblemInstance<Program> {
        private final ControllerSynthesisProblem problem;

        Synthesis(ControllerSynthesisProblem problem) {
            this.problem = problem;
        }

        @Override
        public ControllerSynthesisProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.training().get(0).map();
        }

        @Override
        public void normalise(ObjectNode problemNode) {
            // Maps are given by reference (builtin or generated:...), which is already reproducible.
        }

        @Override
        public Map<String, Object> replay(Program program) {
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("episodes", problem.replay(program));
            out.put("program", problem.view(program));
            out.put("training", problem.evaluateOn(problem.training(), program, false).metrics());
            if (!problem.test().isEmpty()) {
                out.put("test", problem.evaluateOn(problem.test(), program, false).metrics());
            }
            return out;
        }

        /** The hand-written reactive controller on the same training and test episodes. */
        @Override
        public Map<String, Object> reference() {
            Map<String, Object> reactive = new LinkedHashMap<>();
            Evaluation train = problem.evaluateReference(problem.training());
            reactive.put("fitness", train.fitness());
            reactive.put("training", train.metrics());
            if (!problem.test().isEmpty()) {
                Evaluation test = problem.evaluateReference(problem.test());
                reactive.put("testFitness", test.fitness());
                reactive.put("test", test.metrics());
            }
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("reactive", reactive);
            out.put("note", "Hand-written reactive controller (default parameters), same episodes and episode cost, no parsimony term.");
            return out;
        }

        @Override
        public List<Object> describeTargets(SearchResult<Program> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public Program parse(JsonNode genome) {
            return problem.parse(genome.isTextual() ? genome.asText() : genome.path("program").asText());
        }

        @Override
        public boolean success(SearchResult<Program> result) {
            return result.best() != null && result.best().evaluation().metric("successRate", 0) >= 1.0;
        }

        /** Generalisation: the best program on the unseen test maps (no parsimony term). */
        @Override
        public Map<String, Object> bestExtras(Program best) {
            if (problem.test().isEmpty()) {
                return Map.of();
            }
            Evaluation test = problem.evaluateOn(problem.test(), best, false);
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("testFitness", test.fitness());
            out.put("testSuccessRate", test.metric("successRate", 0));
            out.put("test", test.metrics());
            out.put("program", best.toString());
            out.put("pretty", best.pretty());
            return out;
        }
    }

    static final class Navigation implements ProblemInstance<RealVector> {
        private final NavigationProblem problem;

        Navigation(NavigationProblem problem) {
            this.problem = problem;
        }

        @Override
        public NavigationProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.map();
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            Map<String, Object> out = episode(problem.replay(problem.route(solution)));
            out.put("route", ((Map<?, ?>) problem.view(solution)).get("route"));
            return out;
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }

        @Override
        public List<Object> describeTargets(SearchResult<RealVector> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public RealVector parse(JsonNode genome) {
            return realGenome(genome);
        }

        @Override
        public boolean success(SearchResult<RealVector> result) {
            return result.best() != null && result.best().evaluation().metric("success", 0) > 0;
        }
    }

    /** Controller parameter tuning. */
    static final class Tuning implements ProblemInstance<RealVector> {
        private final ControllerTuningProblem problem;

        Tuning(ControllerTuningProblem problem) {
            this.problem = problem;
        }

        @Override
        public ControllerTuningProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.training().get(0).map();
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("episodes", problem.replay(solution));
            out.put("params", ((Map<?, ?>) problem.view(solution)).get("params"));
            out.put("training", problem.evaluateOn(problem.training(), solution).metrics());
            if (!problem.validation().isEmpty()) {
                out.put("validation", problem.evaluateOn(problem.validation(), solution).metrics());
            }
            return out;
        }

        /** The controller's default parameters on training and validation maps (the untuned baseline). */
        @Override
        public Map<String, Object> reference() {
            RealVector defaults = problem.defaults();
            Map<String, Object> baseline = new LinkedHashMap<>();
            Evaluation train = problem.evaluateOn(problem.training(), defaults);
            baseline.put("params", ((Map<?, ?>) problem.view(defaults)).get("params"));
            baseline.put("fitness", train.fitness());
            baseline.put("training", train.metrics());
            if (!problem.validation().isEmpty()) {
                Evaluation val = problem.evaluateOn(problem.validation(), defaults);
                baseline.put("validationFitness", val.fitness());
                baseline.put("validation", val.metrics());
            }
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("defaults", baseline);
            out.put("note", "Untuned controller defaults, evaluated with the same episodes and fitness.");
            return out;
        }

        @Override
        public List<Object> describeTargets(SearchResult<RealVector> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public RealVector parse(JsonNode genome) {
            return realGenome(genome);
        }

        @Override
        public boolean success(SearchResult<RealVector> result) {
            return result.best() != null && result.best().evaluation().metric("successRate", 0) >= 1.0;
        }

        /** Generalisation: the tuned parameters on the held-out validation maps. */
        @Override
        public Map<String, Object> bestExtras(RealVector best) {
            if (problem.validation().isEmpty()) {
                return Map.of();
            }
            Evaluation e = problem.evaluateOn(problem.validation(), best);
            Map<String, Object> out = new LinkedHashMap<>();
            out.put("validationFitness", e.fitness());
            out.put("validation", e.metrics());
            return out;
        }
    }

    /** Warehouse task scheduling. */
    static final class Warehouse implements ProblemInstance<Permutation> {
        private final WarehouseSchedulingProblem problem;

        Warehouse(WarehouseSchedulingProblem problem) {
            this.problem = problem;
        }

        @Override
        public WarehouseSchedulingProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.map();
        }

        @Override
        public Map<String, Object> replay(Permutation solution) {
            return problem.replay(solution);
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }

        @Override
        public List<Object> describeTargets(SearchResult<Permutation> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public Permutation parse(JsonNode genome) {
            return new Permutation(Json.convert(genome, int[].class));
        }

        @Override
        public void normalise(ObjectNode problemConfig) {
            problemConfig.set("map", Json.tree(map()));
            ObjectNode params = problemConfig.get("params") instanceof ObjectNode p ? p : problemConfig.putObject("params");
            params.set("tasks", Json.tree(problem.tasks()));
        }

        /** Success = every task delivered on time. */
        @Override
        public boolean success(SearchResult<Permutation> result) {
            return result.best() != null && result.best().evaluation().metric("lateTasks", 1) == 0;
        }
    }

    /** Multi-robot task allocation. */
    static final class MultiRobot implements ProblemInstance<RealVector> {
        private final MultiRobotAllocationProblem problem;

        MultiRobot(MultiRobotAllocationProblem problem) {
            this.problem = problem;
        }

        @Override
        public MultiRobotAllocationProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.map();
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            return problem.replay(solution.toArray());
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }

        @Override
        public List<Object> describeTargets(SearchResult<RealVector> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public RealVector parse(JsonNode genome) {
            return realGenome(genome);
        }

        @Override
        public void normalise(ObjectNode problemConfig) {
            problemConfig.set("map", Json.tree(map()));
            ObjectNode params = problemConfig.get("params") instanceof ObjectNode p ? p : problemConfig.putObject("params");
            params.set("fleet", Json.tree(problem.fleet()));
        }

        /** Success = a makespan no worse than greedy list scheduling. */
        @Override
        public boolean success(SearchResult<RealVector> result) {
            return result.best() != null && result.best().evaluation().metric("makespan", Double.POSITIVE_INFINITY)
                    <= problem.evaluate(problem.greedy()).metric("makespan", 0) + 1e-9;
        }
    }

    /** Controller failure discovery. */
    static final class Testing implements ProblemInstance<RealVector> {
        private final ControllerTestingProblem problem;

        Testing(ControllerTestingProblem problem) {
            this.problem = problem;
        }

        @Override
        public ControllerTestingProblem problem() {
            return problem;
        }

        @Override
        public WorldMap map() {
            return problem.arena();
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            Scenario scenario = problem.decode(solution);
            ControllerTestingProblem.Execution ex = problem.execute(scenario, problem.controllerId(), true);
            Map<String, Object> out = episode(ex.result());
            out.put("scenario", scenario);
            out.put("verdicts", ex.verdicts());
            out.put("violated", ex.violated());
            out.put("coveredOutcomes", ex.coveredOutcomes());
            return out;
        }

        @Override
        public Map<String, Object> runExtras() {
            Map<String, Object> coverage = new LinkedHashMap<>();
            if (problem.coverage() == null) {
                coverage.put("available", false);
                coverage.put("reason", "controller " + problem.controllerId() + " has no coverage instrumentation");
            } else {
                coverage.put("available", true);
                coverage.put("instrumentation", problem.instrumentation());
                coverage.putAll(problem.coverage().report());
            }
            return Map.of("coverage", coverage);
        }

        @Override
        public Map<String, Object> reference() {
            return null;
        }

        @Override
        public RealVector parse(JsonNode genome) {
            return realGenome(genome);
        }

        /**
         * One failure record per distinct set of violated properties (the first scenario found for
         * each), optionally minimised.
         */
        @Override
        public List<Object> describeTargets(SearchResult<RealVector> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            List<Object> records = new ArrayList<>();
            Set<String> seen = new LinkedHashSet<>();
            for (Candidate<RealVector> target : result.targets()) {
                if (records.size() >= limit) {
                    break;
                }
                Scenario scenario = problem.decode(target.solution());
                List<String> violated = problem.execute(scenario, problem.controllerId(), false).violated();
                String key = String.join(",", violated);
                if (violated.isEmpty() || !seen.add(key)) {
                    continue;
                }
                Map<String, Object> prov = new LinkedHashMap<>(provenance);
                prov.put("evaluation", target.index());
                prov.put("fitness", target.fitness());
                Map<String, Object> minimization = null;
                Scenario recorded = scenario;
                if (minimize) {
                    Failures.Minimized m = Failures.minimize(problem, scenario, violated.get(0));
                    minimization = new LinkedHashMap<>();
                    minimization.put("property", m.property());
                    minimization.put("simulations", m.simulations());
                    minimization.put("steps", m.steps());
                    minimization.put("originalObstacles", scenario.obstacles().size());
                    minimization.put("original", scenario);
                    recorded = m.scenario();
                }
                FailureRecord record = Failures.record(problem, recorded, target.solution(), prov, minimization);
                records.add(record);
            }
            return records;
        }
    }
}
