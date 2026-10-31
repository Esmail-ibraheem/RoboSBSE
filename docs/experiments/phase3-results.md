# Phase 3 results

Measured on 2026-10-27 (engine 0.1.0, Java 21.0.12, Windows 10, 8 logical processors). Unless stated
otherwise, every benchmark used 15 seeds per algorithm, an identical evaluation budget and 8 benchmark
threads. Configurations are in `examples/benchmarks/` and `examples/coverage/`. To reproduce:

```bash
java -jar backend/java-engine/build/libs/robosbse-engine.jar benchmark examples/benchmarks/<file>.json --out experiments/phase3
java -jar backend/java-engine/build/libs/robosbse-engine.jar coverage examples/coverage/coverage-bug3-ga.json --out experiments/phase3
```

Pairwise comparisons are two-sided Mann–Whitney U tests with Holm correction; effect sizes are
Vargha–Delaney A12. For failure discovery the compared measure is **time to target** (evaluations until the
first failure; runs that found none rank after all that did).

## Experiment C (complete): search-based test generation against injected defects

**Question.** Do search-based test generators find controller failures more efficiently than random
testing, and does guidance by behaviour or by code structure help where robustness alone does not?

**Setup.** Testing arena, scenario genome of 30 genes (start, goal, noise, dropout, battery, speed limit,
free-distance setting, four obstacle slots), 2 000 evaluations, fitness = collision robustness (novelty
search uses the controller's behaviour instead: its path, closest approach and episode duration). The first
failure of every run is minimised (ddmin over obstacles, numeric shrinking, rounding).

| Generator | BUG-1 found | median first failure | BUG-2 found | median first failure | BUG-2 obstacles before → after minimisation |
|---|---|---|---|---|---|
| Random testing | 15/15 | 11 | 2/15 | 804 | 1.5 → 1 |
| Hill climbing (stochastic) | 15/15 | 139 | 1/15 | 948 | 4 → 2 |
| Genetic search | 15/15 | 11 | **12/15** | 838 | 2 → 1 |
| Novelty search | 15/15 | 11 | 6/15 | 1048 | 3 → 1 |
| Novelty + fitness (w = 0.5) | 15/15 | 11 | **14/15** | 523 | 2.5 → 1 |

BUG-2 (time to target): GA vs random p(Holm) = 0.006, A12 = 0.17; novelty + fitness vs random
p(Holm) = 3·10⁻⁴, A12 = 0.07; novelty + fitness vs pure novelty p(Holm) = 0.004. GA vs novelty + fitness is
not significant (p(Holm) = 0.33). BUG-1: only hill climbing differs (slower, p(Holm) < 0.001); every
population-based method already hits the defect in its random initial population.

**Interpretation.** BUG-1 (dropout read as free space) is easy: about one random scenario in ten triggers
it. BUG-2 (no braking distance) needs a specific geometry, and there the guidance matters. Random testing and
single-trajectory hill climbing rarely find it. Pure novelty search finds it in 6 of 15 runs: exploring
behaviour helps, but it does not push towards collisions. Blending novelty with the collision robustness
gave the best discovery rate, which is consistent with the novelty literature's advice to combine both, but
not significantly better than the GA here. The GA and random-testing numbers repeat Phase 1 exactly (same
seeds), as they must. Minimisation reduced every failure to at most two obstacles, typically one; BUG-1
failures of the search methods needed no added obstacle at all (the arena's own walls suffice).

### BUG-3: white-box guidance for a rare trigger

BUG-3's unclamped speed boost needs three conditions: battery below 10% during the episode, a speed limit
above 0.8 m/s, and a free straight stretch. With the default genes (battery from 0.05) random testing finds
it at evaluation 33. To make the trigger rare, the battery gene starts at **0.2**, so the faulty branch
is only reached when a scenario drains the battery during the episode (2 of 843 random valid scenarios
reached it). Budget 3 000.

| Oracle | Random testing | GA, robustness (black-box) | GA, branch distance → robustness (white-box) |
|---|---|---|---|
| all safety properties, fitness = min | 1/15 | 0/15 | **6/15** (median first failure 272) |
| velocity only (BUG-3's expected property) | 1/15 | 2/15 | **8/15** (median first failure 345) |

Velocity oracle (time to target): white-box vs random p(Holm) = 0.015, A12 = 0.25; white-box vs black-box
p(Holm) = 0.025, A12 = 0.27. Random vs black-box is not significant (p = 0.58).

**Interpretation.** The `velocity` robustness is +1 for every scenario that does not violate it: a plateau
with no gradient towards low battery, so the black-box GA is no better than random testing. The branch
distance `battery − 0.1` of the instrumented decision `low-battery-boost` provides that gradient. The
white-box GA covered the branch in **all 15 runs**. It then found the failure in 8, because the second
condition (speed limit > 0.8) is not a decision of the controller and again has no gradient. With the
min-aggregated oracle the second stage is additionally masked by the other properties' robustness (≈ 0.2),
which costs two more runs. White-box guidance solves exactly the part of the problem that is a code decision.

### Coverage campaign

`examples/coverage/coverage-bug3-ga.json`: BUG-3 controller, battery ≥ 0.2, GA generator, 6 000 evaluations,
4 evaluation threads.

| | Outcomes covered | Evaluations used |
|---|---|---|
| Targeted campaign | 29 / 30 | 3 127 (the rest was not needed) |
| Random testing, same budget | 29 / 30 | 6 000 |

The first targeted search (for `scan-valid:true`) covered 28 outcomes collaterally. `low-battery-boost:true`
was the one target that needed a real search: covered after 127 evaluations. Neither method covered
`no-free-heading:true`. Its best branch distance stayed at 2.5 m (sensor range minus the largest
free-distance setting), because with four obstacle slots some direction always has a fully open corridor.
The outcome is probably infeasible in this scenario space, which is not proven. For this small controller,
plain random testing already reaches high decision coverage; the targeted search pays off for rare outcomes.

## Memetic algorithm vs GA (obstacle course)

**Setup.** Navigation, obstacle course, 4 waypoints, 3 000 evaluations including every local-search
evaluation. GA with population 40; memetic variants refine 10% of the offspring with 20-evaluation local
searches. A* reference route: 19.5 s.

| Algorithm | Median best time (s) | Goal reached |
|---|---|---|
| GA | 19.60 | 13/15 |
| MA, hill climbing | 19.51 | 15/15 |
| MA, simulated annealing | 19.85 | 13/15 |
| MA, route smoothing (Lamarckian) | **19.49** | 15/15 |
| MA, route smoothing (Baldwinian) | 19.60 | 15/15 |

No pairwise difference in best fitness is significant (smallest p(Holm) = 0.19). The success counts 13/15 vs
15/15 are not significant either (Fisher's exact test, p ≈ 0.48).

**Interpretation.** All variants end at the A* reference level. Local refinement did not change the typical
result, although the three variants with hill-climbing or smoothing local search reached the goal in every
run. A null result on this landscape, reported as such.

## Novelty search vs fitness on the maze

**Setup.** Maze, 10 waypoints, pure-pursuit controller, repair, 6 000 evaluations. The fitness rewards
geodesic progress towards the goal; A* finds a 41.1 s route. Compared measure: behaviour coverage, the share
of a 10 × 10 grid over route *end* positions that the run visited.

| Algorithm | Median behaviour coverage | Goal reached | Median best fitness |
|---|---|---|---|
| GA | 0.07 | 0/15 | 10 305 |
| Novelty search | **0.20** | 0/15 | 10 385 |
| Novelty + fitness (w = 0.5) | 0.15 | 0/15 | 10 256 |
| Random search | 0.10 | 0/15 | 10 409 |

Coverage: novelty vs GA p(Holm) = 3·10⁻⁵ (A12 = 0.99 in favour of novelty); novelty vs random
p(Holm) = 2·10⁻⁵. Novelty vs novelty + fitness is not significant.

**Interpretation.** Novelty search explored nearly three times as many end regions as the GA, and the GA
explored *less* than random sampling: it concentrates on its best basin. But no method reached the goal,
so in this maze, with this fixed-length route encoding and budget, more exploration did not translate into
success (Phase 1 already reported that the GA does not solve this maze). A failure value of about 10 300
means the routes end roughly 30 m of corridor short of the goal.

## Island model vs panmictic GA (maze)

Same maze setup. Panmictic GA of 100, four islands of 25 with ring migration of 2 individuals every 10
generations, and four isolated islands.

| Algorithm | Median best fitness | Goal reached |
|---|---|---|
| Panmictic GA (100) | 10 296 | 0/15 |
| Islands 4 × 25, ring | 10 292 | 0/15 |
| Islands 4 × 25, isolated | 10 304 | 0/15 |

No significant difference (smallest p(Holm) = 0.29). Null result.

## Parallel evaluation: speed-up with identical results

**Setup.** GA (population 40) tuning the reactive controller (three maps per evaluation), 400 evaluations,
5 seeds, evaluation threads 1/2/4/8, runs executed one after another (benchmark threads = 1).

| Evaluation threads | Median wall time | Speed-up | Median CPU time | Median best fitness |
|---|---|---|---|---|
| 1 | 14.2 s | 1.00 | 13.8 s | 30.913 |
| 2 | 7.3 s | 1.95 | 14.2 s | 30.913 |
| 4 | 5.3 s | 2.70 | 19.7 s | 30.913 |
| 8 | 5.7 s | 2.50 | 28.7 s | 30.913 |

Every run's best fitness and convergence curve was identical across thread counts (all 20 runs; p = 1).
Beyond two threads the efficiency drops. Each generation waits for its slowest evaluation (episodes differ
in length), and the machine has 8 logical processors, which may share fewer physical cores (not checked).
CPU time rises accordingly.

## Threats to validity

- One arena, one maze, one obstacle course; 15 seeds per configuration. Null results (memetic, islands) are
  specific to these landscapes and budgets.
- The BUG-3 study changes the battery gene range to make the trigger rare. This is a deliberate test design
  choice, stated above; with the default range the bug is easy for any method.
- Coverage is decision coverage of one hand-instrumented class ([coverage.md](../research/coverage.md)).
- Timings come from a desktop machine with other processes running; the speed-up numbers are indicative.
