# Roadmap

Phases 1 to 5 are complete (see the README). Each later phase builds on interfaces that already exist, so
features can be added without changing the architecture. "Hook" names the existing extension point.

| Course block | Topic | Phase | Status |
|---|---|---|---|
| Local search | random search, random walk, hill climbing, SA | 1 | done |
| Local search | Tabu search | 2 | done |
| Evolutionary | GA | 1 | done |
| Evolutionary | steady-state GA, evolution strategies, PSO | 2 | done |
| Multi-objective | Pareto dominance, non-dominated sorting, NSGA-II, hypervolume | 2 | done |
| SB testing | random / HC / SA / GA test generation, robustness oracles, minimisation, regressions | 1 | done |
| Problems | controller parameter tuning, warehouse task scheduling, exhaustive search | 2 | done |
| SB testing | coverage instrumentation, branch distance, approach level, coverage campaigns, ddmin | 3 | done |
| Hybrid / diversity | memetic algorithms, novelty search, behaviour coverage | 3 | done |
| Parallel | island-model GA, parallel fitness evaluation (identical results for any thread count) | 3 | done |
| Genetic programming | typed robot DSL, subtree operators, bloat control, test-map generalisation | 4 | done |
| GI and repair | constrained source patches, isolated compilation and tests, minimisation, held-out validation | 4 | done |
| Robotics extension | planar arm, inverse kinematics, trajectories, pick and place, multi-robot allocation, dynamic environments | 5 | done |

## Phase 2: expanded optimisation (done)

Implemented: tabu search with move attributes and aspiration; PSO (global/ring); (mu/rho +, lambda)-ES
with self-adaptive or 1/5-rule step sizes; steady-state GA; exhaustive search; permutation representation;
controller parameter tuning over several maps with held-out validation; warehouse scheduling with capacity,
dependencies, deadlines and charging; Pareto dominance, constrained non-dominated sorting, crowding
distance, exact hypervolume, a Pareto archive for every run, NSGA-II; Pareto view and hypervolume
comparisons in the UI. Results: [Phase 2 results](experiments/phase2-results.md).

Left for later: CMA-ES; multiple robots in scheduling; more held-out maps for tuning; preference-based
selection from a Pareto front.

## Phase 3: advanced search and testing (done)

Implemented: memetic algorithm (hill climbing, SA or route-smoothing local search; Lamarckian or Baldwinian
learning; every local evaluation charged); behaviour descriptors for every problem, behaviour coverage for
every run, novelty search with an adaptive archive and optional fitness blending; batch evaluation with an
optional thread pool and bit-identical results for every thread count, CPU-time measurement, synchronous
PSO, island-model GA; hand-placed decision probes in the reactive controllers, branch distance and approach
level, branch-guided failure search, coverage campaigns with target selection and a random baseline; ddmin
and numeric shrinking for failure minimisation; benchmark comparisons by time to target and behaviour
coverage. Results: [Phase 3 results](experiments/phase3-results.md).

Left for later: bytecode instrumentation of arbitrary controllers; descriptors learned or chosen per task;
variable-length route encodings (the maze stays unsolved by every method); asynchronous parallel evaluation
for steady-state algorithms.

## Phase 4: software evolution (done)

Implemented: genetic programming of reactive controllers in a strongly typed language (ramped half-and-half,
subtree/point/constant mutation, subtree crossover, size/depth limits, a per-tick step budget, a parser that
rejects invalid programs, parsimony pressure) with training and unseen test maps and generated environments;
a shared source-patch layer (JavaParser mutation points for comparison, arithmetic and logical operators,
numeric constants and conditions; lexical-preserving application; side-by-side diffs); isolated candidate
execution in child JVMs with heap and time limits; program repair of five faulty controllers (one per defect
kind of the specification) with failing, passing and held-out tests, ddmin patch minimisation, ranking and
comparison with a reference fix; genetic improvement of the correct controller (travel time and counted
work, correctness suite, held-out validation); experiment suites with a combined Markdown report. Results:
[Phase 4 results](experiments/phase4-results.md).

Found along the way: the correct reactive controller can collide when an obstacle corner falls between its
range rays ([formulations, Known limitations](research/problem-formulations.md)). Documented, not fixed,
because a fix changes every earlier result that uses that controller family.

Left for later: fault localisation to focus patch search; richer GP primitives (memory, continuous speeds);
OS-level sandboxing of candidate processes; a fix for the sparse-ray perception defect.

## Phase 5: robotics extensions (done)

- Planar serial arm (two links, and a redundant three-link scene): forward kinematics, joint limits,
  capsule collision checking, a synchronised-move motion model. Problems `arm-ik`, `arm-trajectory` and
  `arm-pick-place` run with every real-vector algorithm (PSO, GA, HC, SA, NSGA-II, ...). References:
  analytical and damped-least-squares IK, a configuration-space A\* planner, greedy planning and exact
  enumeration.
- Multi-robot task allocation (`multi-robot-allocation`): random keys over assignment and order, cost
  model with makespan and distance, greedy list scheduling as reference, simulated execution with
  robot-robot conflict detection.
- Dynamic environments: moving obstacles, timed obstacles, restricted zones and terrain in the simulator,
  sensors and grid planners. Two new maps: `dynamic-obstacle-course` and `delivery-center`.
- UI: Robotic Arm page; multi-robot and dynamic replays in the Laboratory.
- Experiments G (arm), H (multi-robot) and I (dynamic course): [Phase 5 results](experiments/phase5-results.md).
- Methods: [robotics extensions](research/robotics-extensions.md).

Not implemented, left for later: coordination between robots (priorities, reservations, replanning) and
conflict-aware fitness; time-aware planners (space-time A\*); creating dynamic elements in the Environment
Editor; arm dynamics (gravity, torques); a configuration-space planner for more than two links. The
failure-discovery oracle's claim that "safety violations are always defects" holds only for static arenas,
because with moving obstacles a collision can be unavoidable. Dynamic maps are therefore not used for
failure discovery.
