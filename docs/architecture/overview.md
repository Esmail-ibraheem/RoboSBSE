# Architecture overview

RoboSBSE has two processes: a **Java engine**, which owns every computation, and a **Rust UI**, which
configures experiments and visualises them. They talk JSON Lines over the engine's stdin and stdout.

```text
┌──────────────────────────── Rust UI (egui) ────────────────────────────┐
│ pages: Lab · Editor · Testing · Benchmark · History · Algorithms · ... │
│ forms generated from the engine's ParamSpecs · canvas · replay · plots │
└──────────────┬───────────────────────────────────────▲─────────────────┘
   requests    │ start / pause / resume / cancel /      │ events: ready, started,
   (1 per user │ benchmark / simulate / plan /          │ progress (≤10/s), best,
    action)    │ reproduce / get_map / generate_map     │ finished(result), error ...
┌──────────────▼───────────────────────────────────────┴─────────────────┐
│ Java engine (`robosbse-engine serve`)                                  │
│  protocol ─► experiments (ExperimentRunner, BenchmarkRunner, stats)    │
│               │                                                         │
│               ▼                                                         │
│  algorithms ─► SearchContext.evaluate(s) ─► Problem.evaluate(s)         │
│  (RS, RW, HC, SA, GA)   budget · trace ·      │ NavigationProblem       │
│                         archive · targets     │ ControllerTestingProblem│
│                                               ▼                         │
│                         Simulator (deterministic) ◄─ Controller         │
│                         World · RobotSpec · sensors   (correct/faulty)  │
│                         planning: OccupancyGrid, A*, Dijkstra,          │
│                         DistanceField                                   │
└────────────────────────────────────────────────────────────────────────┘
```

The vertical slice: **UI → engine → simulation → fitness → result → robot replay → recorded data.**

1. The UI builds an experiment configuration (`robosbse.experiment/1`) from its forms, inlining the
   current map, and sends `start`.
2. `ExperimentRunner` constructs the `ProblemInstance` and the algorithm (parameters validated against
   their declarations), then runs the algorithm on a worker thread.
3. The algorithm calls `SearchContext.evaluate(solution)` for every candidate. The problem decodes the
   candidate and runs a complete simulation episode. The fitness comes from what happened in that episode.
4. The context streams `progress` and `best` events (throttled), and at the end the runner re-simulates
   the best solution with recording enabled, so that the UI can replay it.
5. The result document is written to `experiments/results/...` and returned in the `finished` event.
   The History page lists those directories.

## Engine packages

| Package | Responsibility |
|---|---|
| `core` | `Problem`, `SolutionSpace`, `MutationOperator`, `CrossoverOperator`, `Neighbourhood`, `Evaluation`, `Candidate`, `Params`/`ParamSpec`, `Budget`, `SearchContext`, `SearchResult`, `RunControl`, `Rng` |
| `core` (multi-objective) | `Pareto` (constrained domination, non-dominated sorting, crowding), `Hypervolume` (exact), `ParetoArchive` |
| `core.representation` | `RealVector`/`RealVectorSpace` (box constraints, block-aware operators) and `Permutation`/`PermutationSpace` (OX/PMX/CX, swap/insert/inversion, enumeration) |
| `algorithms.*` | `baseline` (random search, random walk, exhaustive search), `local` (hill climbing, simulated annealing, tabu search), `evolutionary` (GA incl. steady-state, evolution strategy, PSO, selection), `multiobjective` (NSGA-II), `memetic` (GA + local search), `novelty` (novelty search), `parallel` (island-model GA); `AlgorithmRegistry` with representation compatibility |
| `geometry` | `Vec2`, `Shape` (`Circle`, `Rect`) with signed distance and ray casting, `Angles` |
| `simulation` | `WorldMap` (file format), `World` (time-aware distances and ray casts), `MovingObstacle`, `TimedObstacle`, `Zone`, `RobotSpec`, `Simulator`, `EpisodeConfig`/`EpisodeResult`, `Trajectory`, `MapLibrary`, `MapGenerator` |
| `simulation.controllers` | `Controller` interface, waypoint follower, PID heading, pure pursuit, reactive avoidance, `ControllerRegistry`; `.faulty` holds the injected defects |
| `planning` | `OccupancyGrid`, `GridPlanner` (A\*, Dijkstra, line-of-sight shortcut), `DistanceField` |
| `problems` | `NavigationProblem` (3 objectives), `ControllerTuningProblem`, `WarehouseSchedulingProblem`; `problems.warehouse`: `TaskSet`, `TaskLibrary`; `problems.fleet`: `Fleet`, `MultiRobotAllocationProblem` (random keys, simulated replay, conflict detection) |
| `arm` | `ArmScene` (file format), `ArmKinematics` (forward kinematics, clearance, analytical and damped-least-squares IK), `ArmMotion` (motion model, metrics, replay frames), `ConfigurationSpace` (grid A* motion planner), `ArmIkProblem`, `ArmTrajectoryProblem`, `ArmPickPlaceProblem` |
| `testing` | `Scenario`, `Property`, `Oracle`, `Verdict`, `ControllerTestingProblem`, `FailureRecord`, `Failures` (record, reproduce, ddmin + numeric minimisation) |
| `geneticprogramming` | `Op` (typed primitives), `Node`/`Program` (immutable AST, parser that rejects invalid programs), `ProgramSpace` (ramped half-and-half, subtree/point/constant mutation, subtree crossover, limits), `ProgramController` (interpreter with a step budget), `ControllerSynthesisProblem` |
| `patching` | `Subject` (repair/improvement subjects), `SourceCatalog` (JavaParser mutation points, lexical-preserving patch application, side-by-side diff), `Patch`, `PatchSpace`, `Suites` (tests judged by the reference controller), `WorkCounter`; `patching.sandbox`: `WorkerPool` and `CandidateWorker` (compilation and tests in child JVMs with heap and time limits) |
| `repair` | `RepairProblem` (failing/passing/held-out tests, plausibility, ddmin patch minimisation, validation against the reference fix) |
| `improvement` | `ImprovementProblem` (correctness suite, time and counted-work ratios against the baseline) |
| `testing.coverage` | `Branch`, `Probe`, `CoverageTrace`, `Instrumented`, `BranchFitness` (approach level + branch distance), `CoverageAccumulator` |
| `experiments` | `Problems` (factory), `PatchInstances`, `ArmInstances`, `ExperimentRunner`, `BenchmarkRunner`, `CoverageCampaign`, `SuiteRunner` (experiment automation), `Statistics` |
| `protocol` | `Json`, `Capabilities`, `ProtocolServer` |
| `cli` | `Main` (`serve`, `run`, `benchmark`, `coverage`, `suite`, `reproduce`, `plan`, `generate-map`, `capabilities`) |

## Key design decisions

**One evaluation gateway.** Algorithms never call the fitness function directly. `SearchContext.evaluate`
does the following:

- checks pause and cancel;
- enforces the evaluation budget and the optional wall-clock limit by throwing `SearchTermination`;
- applies the problem's (Lamarckian) repair;
- updates best-so-far, the convergence step curve, the archive of distinct elites and the target hits;
- notifies observers.

Algorithms therefore read like textbook pseudo-code, can never exceed their budget, and are compared
on exactly equal terms. Experiments compare *fitness evaluations*, not iterations or generations.

**Multi-objective bookkeeping in the same gateway.** For problems that declare objectives, the context also
feeds every evaluation into a bounded Pareto archive of non-dominated feasible solutions, and records the
archive's hypervolume as a series. Single-objective algorithms and NSGA-II are therefore compared on the
same indicator without any change to their code.

**Batches, parallel evaluation and identical results.** `SearchContext.evaluateAll` lets population-based
algorithms hand over a whole generation. With several evaluation threads only repair + fitness run
concurrently; the bookkeeping then runs on the calling thread in batch order with the same stopping rules as
one-by-one evaluation. Results are therefore bit-identical for every thread count (tested per algorithm).
The context also records CPU time and the behaviour coverage of every run ([island-model.md](../algorithms/island-model.md)).

**Coverage probes return the condition.** The reactive controllers' decisions are wrapped in probe calls that
return the original condition and record outcome and branch distance only when a trace is attached, so
instrumentation cannot change behaviour ([coverage.md](../research/coverage.md)).

**Untrusted code runs in other processes.** Genetic programs are data interpreted by the engine (no loops,
bounded size, a per-tick step budget). Source patches are different: they become real Java classes, so
they are compiled and executed only in child JVMs with heap and time limits, killed and replaced on
timeout or crash ([program-repair.md](../research/program-repair.md)). Problems that own such processes
implement `AutoCloseable`, and every place that creates a problem instance closes it.

**Representation-aware compatibility.** Problems declare their representation and operators; algorithms
declare the representations they support and whether they need objectives. The runner refuses invalid
pairs (e.g. PSO on permutations, NSGA-II on failure discovery) with an explicit message, and the UI only
offers valid ones.

**Determinism and reproducibility.** All randomness comes from explicitly seeded `Rng`s (SplittableRandom).
The search uses the experiment seed. Sensor noise uses a separate, problem-level seed with a fixed number
of draws per step. Every result embeds its full configuration, including the inlined map. Re-running the
`experiment` object of a result gives identical numbers, and `ExperimentRunnerTest` verifies this.

**Common random numbers in testing.** Every candidate scenario sees the same underlying noise and dropout
draws. A scan is lost when `u_t < p`, so more dropout always means a superset of lost scans. That makes the
robustness landscape smooth in the noise and dropout genes, and it is the reason search can climb towards a failure.

**Fitness is minimised everywhere.** Problems that maximise something (clearance, robustness margins)
transform it and document the transformation.

**Search stays in Java.** The UI never evaluates candidates. Progress events are throttled (≤10 per second,
best-so-far snapshots ≤7 per second), so even 10⁶-evaluation runs cost only a few messages per second.

**Extensibility.** A new algorithm is a class plus one `AlgorithmRegistry` entry with its `ParamSpec`s.
A new representation implements `SolutionSpace<S>`, and every algorithm that needs only the operators it
provides works on it. The UI builds forms from the capability message, so neither change needs UI code.

**Honest UI.** Every control triggers real engine work. What is not implemented (e.g. multi-robot
coordination, creating dynamic obstacles in the editor) is listed as such on the Dashboard instead of
being offered as an inert control.

## Threading

- **Engine:** the main thread reads requests. Each run or benchmark executes on a pooled worker thread.
  Benchmarks may use their own fixed pool (`threads`). A run with `options.threads > 1` owns a fixed pool
  of daemon evaluation threads, shut down when the run ends. Each run has its own seeded RNG and
  bookkeeping is sequential, so thread scheduling does not change any result. `ExperimentRunnerTest.benchmarkComparesAlgorithmsOnEqualBudgets` checks this by repeating a 2-thread benchmark.
- **UI:** a reader thread parses engine stdout into a channel and requests a repaint. The UI thread drains
  the channel each frame and never blocks.

## File formats

| Schema | Producer | Content |
|---|---|---|
| `robosbse.map/1` | editor, `examples/maps` | width, height, start pose, goal, tolerance, obstacles (`circle`/`rect`); optional `movingObstacles`, `timedObstacles`, `zones` (omitted when empty, so static maps keep the original format) |
| `robosbse.arm/1` | `examples/arm` | planar arm scene: workspace, base, links, joint limits, link radius, joint speed, home pose, obstacles, target, pick-and-place jobs |
| `robosbse.fleet/1` | `examples/fleet` | multi-robot instance: map, robot depots, speed, service time, return-home flag, transport jobs |
| `robosbse.experiment/1` | UI, `examples/experiments` | name, seed, budget, problem (type, map, params, controllerParams, robot), algorithm (type, params), options |
| `robosbse.result/1` | engine | the above, normalised, plus stop reason, best, convergence, series, stats, archive, targets, replay, reference, failures |
| `robosbse.benchmark/1` / `robosbse.benchmark-result/1` | `examples/benchmarks` / engine | algorithms × seeds, summary, pairwise statistics, median convergence |
| `robosbse.failure/1` | engine | self-contained failing scenario: arena, scenario, controller, testing settings, robot, verdicts, differential reference, provenance, minimisation |
| `robosbse.coverage-campaign/1` / `robosbse.coverage/1` | `examples/coverage` / engine | campaign: testing problem, generator, budget, seed, threads; report: per-target status, evaluations, first hit, approach level, branch distance, covering scenario, scenario counts, random baseline |
| `robosbse.repair-subject/1` | `examples/faulty-controllers/repair` | Java subject for repair or improvement: source path, class, scope, defect, reference fix, failure records for search and held-out tests |
| `robosbse.suite/1` / `robosbse.suite-result/1` | `examples/suites` / engine | list of benchmarks and coverage campaigns; combined result plus `report.md` |
| `robosbse.tasks/1` | `examples/warehouse` | warehouse order book: map, depot, charger, capacity, speeds, battery, orders with deadlines and dependencies (inlined into scheduling results) |

Unknown fields are ignored on both sides, so the formats can grow without breaking older readers.
