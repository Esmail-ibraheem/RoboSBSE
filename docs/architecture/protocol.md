# UI ↔ engine protocol (version 1)

Transport: **JSON Lines** over the engine's stdin (requests) and stdout (responses and events), started as
`java -jar robosbse-engine.jar serve`. The engine writes logs and stack traces to stderr, and the UI shows
them under *Settings*.

Every message is one JSON object on one line, with:

- `"v"`: protocol version (currently `1`). A request with another version is answered with `unsupported_version`.
- `"type"`: message type.
- `"id"`: chosen by the client for requests. Every response or event caused by a request echoes the same id.

Numbers that are NaN or infinite are written as `null`.

## Requests (UI → engine)

| type | fields | answer |
|---|---|---|
| `hello` | – | `hello` with the capability document (same content as `ready`) |
| `get_map` | `map`: `"builtin:<name>"`, a file path or an inline map | `map` `{map}` |
| `generate_map` | `params`: `{kind: "scatter"\|"maze", seed, count, width, height, cellsX, cellsY, cellSize, wall}` | `map` `{map}` |
| `start` | `experiment` (`robosbse.experiment/1`), optional `output` directory | `started`, then `progress`/`best`…, then `finished` `{result}` or `error` |
| `pause` / `resume` / `cancel` | `id` of an active run or benchmark | `state` `{state: paused\|running\|cancelling}`; a cancelled run still ends with `finished` (`stopReason: CANCELLED`) |
| `benchmark` | `benchmark` (`robosbse.benchmark/1`), optional `output` | `started`, `benchmark_progress`… then `benchmark_finished` `{result}` |
| `coverage` | `campaign` (`robosbse.coverage-campaign/1`), optional `output` | `started`, `coverage_progress`… then `coverage_finished` `{result}` (`robosbse.coverage/1`); cancel with `cancel` |
| `simulate` | `problem` + `genome: [...]` (any problem; the array of every solution view) or `route: [[x,y],...]` (navigation) | `simulation` `{fitness, metrics, objectives, solution, replay}`; the `replay` has the same format as in results |
| `plan` | `problem` (any) | `plan_result` `{reference}`: `{astar, dijkstra}` for navigation, `{greedy}` for scheduling, `{defaults}` for tuning |
| `reproduce` | `failure` (`robosbse.failure/1`) | `reproduction` `{reproduced, expected, violated, referenceViolated, outcome, verdicts, trajectory}` |
| `shutdown` | – | `bye`, then the engine cancels runs and exits |

## Events (engine → UI)

| type | when | fields |
|---|---|---|
| `ready` | once at start-up | `engine, version, protocol, java, algorithms[] (with representations, multiObjective), problems[] (with representation, operators, objectives), controllers[], properties[], maps[], taskSets[], robot, operators` |
| `started` | run accepted | `maxEvaluations` (runs) |
| `progress` | at most every 100 ms, and on the last evaluation | `evaluations, maxEvaluations, iteration, elapsedMs, bestFitness, currentFitness, targets, firstTarget, stats{}, convergence[[eval, best]...]` (only the points since the previous event) |
| `best` | new best, at most every 150 ms | `evaluation, fitness, metrics{}, solution` (navigation: `{route, genome}`; testing: `{scenario, genome}`) |
| `finished` | run ended | `result` (`robosbse.result/1`) |
| `benchmark_progress` | one run of a benchmark finished | `finished, total, run{algorithm, seed, bestFitness, success, ...}`. Events may arrive out of order, so clients keep the maximum `finished` |
| `benchmark_finished` | benchmark ended | `result` (`robosbse.benchmark-result/1`) |
| `coverage_progress` | one coverage target handled | `finished, total, target{label, decision, outcome, status: targeted\|collateral\|uncovered\|not-run, evaluations, firstEvaluation, approachLevel, branchDistance, scenario}` |
| `coverage_finished` | campaign ended | `result` (`robosbse.coverage/1`) |
| `error` | any failure | `code` (`invalid_json`, `unknown_command`, `unsupported_version`, `invalid_request`, `run_failed`, `benchmark_failed`, `coverage_failed`, `unknown_run`, `failed`), `message` |

## Capability entries

```json
{"id": "genetic-algorithm", "name": "Genetic Algorithm", "family": "evolutionary", "summary": "...",
 "params": [{"name": "populationSize", "type": "INT", "defaultValue": 50, "min": 2, "max": 10000,
             "options": [], "description": "Number of individuals."}, ...],
 "requires": ["random", "mutation", "crossover"]}
```

Results of multi-objective problems carry `pareto: {objectives, referencePoint, hypervolume, points[]}`;
tuning results carry `bestExtras: {validationFitness, validation}`. Every result carries `cpuMs` and
`threads`, and `stats.behaviourCoverage` for problems with behaviour descriptors. Testing results carry
`coverage` (`available`, `instrumentation`, `outcomes`, `covered`, per-decision scenario counts), and testing
replays list `coveredOutcomes`. Controllers with instrumentation list their `branches[]` (`id`,
`description`, `parent`) and `instrumentation` in the capabilities.

Phase 4 problem types: `controller-synthesis` (representation `program`; the solution view has
`program` (s-expression), `pretty`, `size`, `depth`; replays carry per-map `episodes` like tuning;
`bestExtras` has `testFitness`, `testSuccessRate`; `reference.reactive` is the hand-written controller),
`program-repair` and `genetic-improvement` (representation `patch`; the genome is the patch text
`point:choice,...`). Repair results carry `patches[]` (ranked, minimised, with `edits`, held-out validation,
`equivalentToReferenceFix`, `identicalToReferenceFix`, `changes` and a full side-by-side `diff`) and
`repair` (subject, test counts, candidate statistics, original source); improvement results carry
`improvements[]` and `improvement`. The capabilities list `subjects[]` (id, name, purpose, defect).

Phase 5 problem types (all `real-vector`): `arm-ik`, `arm-trajectory` and `arm-pick-place` take the
scene as `problem.scene` (a built-in name or an inline `robosbse.arm/1` object). Their replays carry
`frames[]` (`t`, joint `points`, `clearance`, `segment`), `keyframes`, `metrics` and the inlined `scene`;
pick-and-place replays add `plan` and `carrying` (object carried into each keyframe). References:
`analytical`/`numerical` (IK), `straight[]` with `genome` or planned `frames` (trajectory), `greedy` and
`optimum` with `genome` (pick and place), and `plannerExpansions`. `multi-robot-allocation` takes
`params.fleet`; its replay has `robots[]` (`tasks`, `route`, `trajectory`, `simulatedTime`),
`conflicts[]` (`robots`, `start`, `end`, `x`, `y`, `closestDistance`), `stops` and `metrics`
(`simulatedMakespan`, `conflicts`, `failedLegs`). The capabilities list `armScenes[]` (full scenes) and
`fleets[]`.

Experiments accept `options.threads` (evaluation threads). Benchmarks accept `compareBy` (`bestFitness`,
`hypervolume`, `behaviourCoverage`, `timeToTarget`, `testFitness`), per-algorithm `options` and `problemParams` overrides
(the latter not together with `compareBy: bestFitness`), and `describeFailures` (minimised first failure per
run, summarised as `minimisedFailures`).

Parameter types are `INT`, `DOUBLE`, `BOOL`, `CHOICE` (with `options`) and `STRING`. The engine rejects
unknown or out-of-range parameters when a run starts, so typos fail fast instead of silently using defaults.

## Example session

```json
→ {"v":1,"type":"start","id":"lab-1","experiment":{"name":"demo","seed":42,"budget":{"maxEvaluations":2000},
    "problem":{"type":"navigation","map":"builtin:warehouse","params":{"waypoints":5}},
    "algorithm":{"type":"genetic-algorithm","params":{"populationSize":40}}}}
← {"v":1,"type":"started","id":"lab-1","maxEvaluations":2000}
← {"v":1,"type":"best","id":"lab-1","evaluation":1,"fitness":10433.2,"metrics":{...},"solution":{"route":[[...]]}}
← {"v":1,"type":"progress","id":"lab-1","evaluations":412,"bestFitness":35.1,"convergence":[[345,41.2],[401,35.1]],...}
← {"v":1,"type":"finished","id":"lab-1","result":{"schema":"robosbse.result/1","stopReason":"BUDGET_EXHAUSTED",...}}
```

`ProtocolServerTest` (Java) drives the real server over pipes, and `engine_tests.rs` (Rust) drives the real
engine process through the UI's client. Both test this catalogue end to end.
