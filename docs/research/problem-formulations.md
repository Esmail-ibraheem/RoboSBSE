# Problem formulations, simulation assumptions and testing methodology

## 1. Simulation model and assumptions

| Aspect | Model |
|---|---|
| World | `[0, W] × [0, H]` m. The border acts as a wall. Static obstacles are circles and axis-aligned rectangles |
| Robot | Disc of radius 0.25 m, unicycle kinematics with state `(x, y, θ, v, ω)` |
| Integration | Fixed step `dt = 0.05 s`, midpoint rule: `θ_m = θ + ω·dt/2`, `x += v·cos θ_m·dt`, `y += v·sin θ_m·dt` |
| Actuation | Command `(v_cmd, ω_cmd)` saturated to `v ∈ [0, 1] m/s`, `\|ω\| ≤ 2 rad/s`, then rate-limited by `a ≤ 1 m/s²` and `α ≤ 8 rad/s²` (the turning constraints) |
| Sensing | 24 rays over 360°, range 4 m, exact ray casting plus `N(0, σ²)` noise on hits. A whole scan is lost with probability `p` (all readings `NaN`) |
| Energy | `P = 2 + 10\|v\| + 2\|ω\| + 4\|a\|` W from a 1500 J battery. Depletion stops the robot |
| Termination | goal reached (automatic or by report), collision (body overlaps an obstacle), out of bounds, no progress > 0.15 m within 8 s (deadlock), battery empty, invalid command (NaN/∞), controller exception, false completion report, step limit |
| Determinism | Same inputs and seeds give an identical episode. Sensor randomness comes from a dedicated stream with a fixed number of draws per step (common random numbers) |

Simplifications, deliberately chosen for Phase 1: perfect localisation, no wheel slip, no dynamics beyond
rate limits, and collision checks at step resolution. Phase 5 added moving and timed obstacles, restricted
zones and rough terrain ([robotics extensions, §3](robotics-extensions.md)); static maps behave exactly as
before. At 1 m/s a step moves 5 cm,
which is less than the robot's radius, so a step cannot pass through an obstacle thicker than 10 cm.

Measured episode properties: travel time, distance, energy, collisions, minimum obstacle and border
clearance, smoothness (`Σ|ω_k − ω_{k−1}|`), success, final distance, maximum commanded speed and turn rate,
command-bound violations, lost scans, completion report and its distance.

## 2. Route optimisation (`navigation`)

**Search space.** `K` intermediate waypoints `w_1 … w_K ∈ [r, W−r] × [r, H−r]`, encoded as a real vector of
length `2K` with block size 2. The executed route is `start → w_1 → … → w_K → goal`.

**Constraint violation.** `V = ∫_route max(0, (r + 0.05) − clearance(p)) ds`, integrated numerically every
5 cm on the interpolated clearance grid. It grows with both the depth and the length of an intrusion, so it
gives a gradient out of obstacles rather than a binary penalty.

**Fitness (minimised), from the simulated episode:**

```
success:  f = objective + 50·V
failure:  f = 10 000 + 10·D_geo(end position) + 50·V
objective ∈ { time, energy, travelled length, w_t·time + w_e·energy + w_l·length + w_s·smoothness }
```

`D_geo` is the obstacle-aware geodesic distance to the goal (Dijkstra distance field). The constant offset
makes every success better than every failure. The progress term ranks failures by how far the robot got,
which Euclidean distance would get wrong in mazes.

**Repair (optional, Lamarckian).** A waypoint inside the inflated obstacles is pushed along the clearance
gradient until it is free (at most 6 iterations), and the repaired genome replaces the original.

**Target (recorded, not stopping).** The first successful route, giving the "evaluations to first feasible
solution" metric.

**Behaviour descriptor** (novelty search, behaviour coverage): final position and positions at 75%, 50% and
25% of the episode, divided by the map size. **Route smoothing** is the problem's own neighbourhood for memetic
local search: a move pulls one waypoint the fraction α of the way to the midpoint of its route neighbours.

**Objectives (multi-objective search).** `(time, energy, risk)`, all minimised, with collision risk
`risk = exp(-max(0, min clearance) / 0.2 m)`: 1 at contact, 0.08 at 0.5 m clearance. Feasible means the
robot arrives and the planned polyline respects the inflated obstacles (`V = 0`). Otherwise the constraint
violation is `V + 1 + D_geo(end)/D_geo(start)`, so failures closer to the goal violate less. The hypervolume
reference point is `(episode limit, battery capacity, 1)`, or `referenceTime` / `referenceEnergy` when set.
The weighted objective has a `weightRisk` term, used for the weighted-sum baseline of Experiment D.

## 3. Controller failure discovery (`controller-testing`)

**System under test.** A reactive controller that gets only the pose, the goal and range readings
(no map). It must reach the goal and *report* completion. The simulator ends the episode only on that report.

**Scenario space.** `10 + 5·M` bounded genes:

| genes | meaning | default range |
|---|---|---|
| 0–2 | start pose `x, y, θ` | inside the arena, `θ ∈ [−π, π]` |
| 3–4 | goal `x, y` | inside the arena |
| 5 | sensor noise σ | [0, 0.03] m |
| 6 | scan dropout probability | [0, 0.6] |
| 7 | initial battery | [0.05, 1] |
| 8 | controller speed limit | [0.3, 1.0] m/s |
| 9 | controller free-distance setting (configuration under test) | [0.4, 1.5] m |
| 10+5k … 14+5k | obstacle slot k: `x, y, size, shape (<0.5 circle), active (≥0.5)` | size [0.2, 0.8] m |

**Decoding with repair.** Slot obstacles that would overlap the start (keep-out `r + 0.15`) or the goal are
dropped. A scenario is **invalid** when the start or goal overlaps the base arena or start and goal are less
than 2 m apart. An invalid scenario's fitness is `2 + violation`, which is worse than any valid one.

**Oracle.** Seven properties, each with a robustness `ρ ∈ [−1, 1]`. `ρ < 0` if and only if the property is
violated; this equivalence is tested on random scenarios.

| property | ρ | violated when |
|---|---|---|
| `collision` | `min clearance / r` | the body penetrates an obstacle |
| `bounds` | `min border clearance / r` | the body leaves the world |
| `valid-command` | ±1 | a NaN/∞ command or a controller exception |
| `velocity` | +1, or `−0.01 − excess/v_max` | a command exceeds the robot's limits |
| `completion` | `(tol − d_report)/tol`, +1 if never reported | completion reported outside the goal tolerance |
| `clearance` (optional) | `(min clearance − c_min)/r` | clearance below `c_min` (0.03 m) |
| `liveness` (optional) | +1, or `−remaining/initial` geodesic distance | goal not reached although it is reachable and the battery suffices |

**Fitness.** `fitness = min ρ` over the selected properties (`fitness = min`), or the robustness of a single
guiding property. Single-property guidance exists because the properties have different scales. Under the
min, a correctly reported completion (ρ ≈ 0.2) masks the collision gradient as soon as the clearance
exceeds 0.05 m. Collision-guided search finds BUG-2 in 80% of runs; min-guided search would plateau at 0.2.
In both modes, a violation of *any* selected property counts as a discovered failure.

**White-box fitness (Phase 3).** The reactive controllers are instrumented with decision probes
([coverage.md](coverage.md)). `fitness = branch:<decision>:<true|false>` first guides the search to that
decision outcome and then to a violation: `1 + approachLevel + d/(d+1)` while the outcome is uncovered,
`ρ/(1 + |ρ|)` once it is covered (negative = violation). Invalid scenarios then score `100 + violation`.
Every testing run also reports the branch coverage of all scenarios it generated.

**Behaviour descriptor:** the controller's path (as for navigation, normalised by the arena), its closest
approach to an obstacle (clamped to [0, 1] m) and the episode duration as a fraction of the step limit.

**Distinguishing defects from impossible tasks.**
- Safety properties: obstacles are static and every valid scenario starts collision-free. A disc robot can
  always avoid a collision by stopping and turning in place, so every safety violation is a defect of the
  controller.
- Liveness: applies only when the planner confirms the goal is reachable for the robot's footprint *and* an
  energy estimate (`1.3 · path length / speed · P(speed)`) fits the battery. Otherwise the verdict is
  "not applicable".
- Differential check: each failure record also runs the correct controller on the same scenario. All four
  saved regressions pass on the correct controller, and the correct controller showed no safety violation
  in 36 000 GA-generated scenarios and 2 000 random ones during development (its worst clearance was 0.08 m).
  Absence of evidence was not evidence of absence: Phase 4 found three scenarios in which it does collide
  (see Known limitations).

**Failure reduction (Phase 3).** Every change is kept only if the scenario stays valid, its obstacles keep the
start/goal keep-out, and the controller still violates the same property:
1. **ddmin** (Zeller and Hildebrandt, 2002) over the obstacle set: subsets and complements at increasing
   granularity, giving a 1-minimal set (removing any single remaining obstacle makes the failure disappear);
2. **numeric shrinking**: scan dropout and sensor noise towards 0, the initial battery towards full. First
   the neutral value itself, otherwise 10 bisection steps to the still-failing value closest to it;
3. **rounding** of all positions, sizes and angles to 0.1, else 0.01.
The minimised scenario is stored together with the original and a log of every accepted step. (Phase 1 used
a greedy one-obstacle-at-a-time pass with all-or-nothing numeric resets; the four saved regression records
were produced by that version and still reproduce.)

**Failure records** (`robosbse.failure/1`) contain everything needed for replay: arena, scenario, controller
and its parameters, testing settings including the noise seed, robot specification, verdicts, provenance
(experiment, algorithm, seed, evaluation index) and minimisation log. `robosbse-engine reproduce` replays a
record; exit code 0 means every recorded violation occurred again.

## 4. Controller parameter tuning (`controller-tuning`)

**Behaviour descriptor:** per training episode, duration / step limit and minimum clearance (clamped to [0, 1] m).


**Search space.** The selected controller parameters, each inside a documented tuning range
(`ControllerTuningProblem.RANGES`). Defaults per controller:

| Controller | Tuned parameters |
|---|---|
| `pid-heading` | kp [0.2, 8], ki [0, 1], kd [0, 1], cruiseSpeed [0.3, 1], switchRadius [0.1, 1], turnInPlaceAngle [0.2, 2.5] |
| `waypoint-follower` | kHeading, cruiseSpeed, switchRadius, turnInPlaceAngle, approachGain [0.2, 3] |
| `pure-pursuit` | lookahead [0.2, 2], cruiseSpeed, approachGain |
| `reactive` | speedLimit [0.3, 1], margin [0.05, 0.5], kHeading, freeDistance [0.4, 2], approachGain |

**Episodes.** One per training map. Route-following controllers follow the map's smoothed A* route
(computed once); the reactive controller drives to the goal with its sensor only. Each evaluation runs every
training episode in full.

**Fitness.** Per episode, the objective (time, energy, or `w_t·time + w_e·energy + w_r·risk + w_s·smoothness`)
on success, else `1000 + 10 · D_geo(end)`. Aggregated by the **mean**, or the **worst case** (`aggregate = max`).
Objectives `(mean time, mean energy, mean risk)`. Feasible means every training episode succeeds.

**Generalisation.** Validation maps are never used during search. The result reports the best parameters'
performance on them (`bestExtras.validationFitness`, also per benchmark run), and the untuned defaults on
training and validation maps as the conventional reference.

## 5. Warehouse task scheduling (`warehouse-scheduling`)

**Instance (`robosbse.tasks/1`).** A map, a depot, a charger, the robot's carrying capacity, planning speed,
service time per stop, charging rate, initial battery, a battery reserve, and the orders: pickup point,
delivery point, deadline, and prerequisite orders (`after`). Bundled: `orders-4` (8 stops, exhaustively
solvable) and `orders-8` (16 stops, capacity 2, three dependencies, 35% battery).

**Search space.** Permutations of the 2n stops, read as a priority list.

**Decoder.** Repeatedly take the first listed stop that is feasible now:
- a pickup needs free capacity and all prerequisite orders delivered;
- a delivery needs its pickup done.

Before each leg, if the battery could not cover *reach the stop + serve it + reach the charger + reserve*,
the robot first drives to the charger and charges fully. Every decoded schedule therefore satisfies
precedence, capacity, dependency and battery constraints by construction. Only deadlines can be violated:
they count as tardiness, or as constraint violation with `hardDeadlines`. The instance is validated up
front: unknown or cyclic dependencies, unreachable stops, and stops that cannot be served on a full battery
are all rejected.

**Cost model.** Geodesic stop-to-stop distances (Dijkstra on the inflated grid), travel time = distance /
planning speed, plus service and charging times. Energy follows the robot's power model at the planning
speed. Objectives `(makespan, tardiness, energy)`; fitness `makespan + w_T·tardiness + w_D·distance + w_E·energy`
(or one of them alone). Reference: greedy dispatching (always the nearest feasible stop). The hypervolume
reference point is 1.5 × the greedy objectives (+60 s tardiness).

**Behaviour descriptor:** `(makespan, tardiness, energy)` divided by the hypervolume reference point.

**Validation by simulation.** Replays execute the mission leg by leg with the waypoint follower along A*
routes, with battery state carried over and service and charging shown as pauses. They report simulated
against modelled travel time; the measured deviation was −1.8% for one 16-stop schedule.

## 6. Controller synthesis, program repair, genetic improvement (Phase 4)

These problems are described with their methods: [genetic programming](../algorithms/genetic-programming.md)
(typed controller programs, training and unseen test maps), and [program repair and genetic
improvement](program-repair.md) (source patches to Java controllers, isolated execution, tests judged by
the reference controller, held-out validation).

## 6a. Evaluation metrics

| Metric | Where |
|---|---|
| Final best fitness (per run) | results, benchmark summaries, box plots |
| Best-so-far after k evaluations | exact step curve (`convergence`), median and quartiles in benchmarks |
| Success rate | navigation: a successful route found; testing: at least one failure found |
| Evaluations to first target | first feasible route or first failure (`firstTargetEvaluation`) |
| Wall-clock and CPU time | per run, excluding pauses; CPU time sums the running thread and all evaluation threads |
| Behaviour coverage | fraction of a 10 × 10 grid over the first two behaviour components that the run visited |
| Branch coverage | covered decision outcomes of an instrumented controller (testing runs, coverage campaigns) |
| Time to target | evaluation of the first target hit; runs without one are ranked after all that found one |
| Constraint violations | `constraintViolation`, `violation` metric |
| Statistical comparison | two-sided Mann–Whitney U (normal approximation with tie and continuity correction, matches SciPy), Holm–Bonferroni correction, Vargha–Delaney A12 with conventional magnitude thresholds; on best fitness, hypervolume, behaviour coverage or time to target (`compareBy`). Benchmarks whose algorithms override problem parameters (e.g. a different fitness function) may not compare raw best fitness |
| Pareto quality | normalised exact hypervolume of the archive of non-dominated feasible solutions, with its reference point |
| Generalisation | best solution's fitness on held-out validation maps (controller tuning) |

## 6b. Robotic arm, multi-robot allocation, dynamic environments (Phase 5)

Described in [robotics-extensions.md](robotics-extensions.md): the planar arm model and the `arm-ik`,
`arm-trajectory` and `arm-pick-place` problems with analytical, numerical, configuration-space and
enumeration references; `multi-robot-allocation` with simulated execution and conflict detection; and the
semantics of moving obstacles, timed obstacles and zones.

## 7. Known limitations

- Fixed-length route encoding. The required number of waypoints is map-dependent; the maze needs about 10,
  and the GA did not solve it in the example run (`nav-maze-ga.json`: seed 42, 6 000 evaluations).
- Hypervolume values depend on the reference point. They are comparable only within one problem
  configuration, and every result records its reference point.
- The scheduling cost model ignores turning time and controller dynamics. The replay quantifies the gap
  but does not feed back into the fitness.
- Tuning generalisation is measured on few held-out maps (one in Experiment B).
- The `velocity` and `completion` robustness values give no gradient towards *triggering* conditions
  (plateaus). Branch distance supplies one for conditions that are decisions of the controller (BUG-3's
  battery check), but not for conditions spread over several variables: once the faulty branch is reached,
  the speed-limit condition of BUG-3 is still unguided.
- Minimisation does not simplify trajectories, and bisection finds *a* still-failing boundary value, not
  necessarily the smallest one when failure is not monotone in the value.
- Coverage is decision coverage of hand-instrumented controller classes only ([coverage.md](coverage.md)).
- **The correct reactive controller is not collision-free (found in Phase 4).** Section 3 argues that every
  safety violation is a controller defect, and earlier phases found none for the correct controller. While
  generating failing tests for repair subject RB1, the search found three scenarios in which the correct
  reactive controller penetrates an obstacle by 0.6–1.4 cm (`examples/faulty-controllers/known-issues/`).
  Cause, verified by varying one factor at a time: its safety checks only know the obstacle points hit by the
  24 range rays (15 degrees apart), so a rectangle corner between two rays is invisible. One case collides even
  with perfect sensing; scan dropout makes it more likely; with 64 rays all three pass. The defect is
  documented, not fixed: fixing it changes every earlier result that uses the reactive controller family.
  Tests of the repair and improvement modules exclude scenarios in which the reference violates a property.
- No failures being found is not evidence of correctness.
- Phase 5: the grid planners ignore moving obstacles, multi-robot replays report robot-robot conflicts
  without resolving them, and the arm model is kinematic ([robotics extensions, §4](robotics-extensions.md)).
