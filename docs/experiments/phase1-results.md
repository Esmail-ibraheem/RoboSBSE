# Phase 1 results

Measured on 2026-10-27 with engine 0.1.0 and Java 21.0.12 on a Windows 10 desktop, 8 worker threads.
Each benchmark ran 15 seeds per algorithm with an identical evaluation budget for every algorithm.
Configurations are in `examples/benchmarks/`. To reproduce:

```bash
java -jar backend/java-engine/build/libs/robosbse-engine.jar benchmark examples/benchmarks/<file>.json --out experiments/benchmarks
```

Thread scheduling does not affect any per-run result, because each run has its own seeded generator.
Wall-clock times do vary.

## Experiment A: navigation optimisation

**Question.** Which metaheuristic produces the best route under a fixed budget of 2000 fitness evaluations?
Each evaluation is a full simulation of the waypoint follower driving the candidate route.

### Obstacle course (4 waypoints, objective = travel time)

| Algorithm | Median best fitness | Success rate | Median evals to first feasible route | Best run |
|---|---|---|---|---|
| Random search | 10 119.8 | 20% | 1146 | 26.45 s |
| Hill climbing (first improvement, restarts, step 0.03) | 10 113.5 | 27% | 1462.5 | 25.05 s |
| Simulated annealing (calibrated T0, budget-derived geometric cooling) | 10 151.9 | 40% | 970.5 | 18.88 s |
| **Genetic algorithm** (pop 40, tournament 3, one-point, Gaussian) | **19.60 s** | **93%** | 267 | 18.90 s |
| A\* reference (grid 0.1 m, smoothed, same controller) | 19.50 s | – | 4 488 node expansions | – |

Fitness above 10 000 means the best route still failed (penalty plus geodesic distance left). The
"median evals to first feasible route" column only covers runs that found one (3, 4, 6 and 14 of 15 runs).
GA vs random search: p(Holm) = 8·10⁻⁵, A12 = 0.03. GA vs SA: p(Holm) = 0.006, A12 = 0.16 (both large effects).

### Warehouse (5 waypoints)

| Algorithm | Median | Success | A12 vs GA |
|---|---|---|---|
| Random search | 10 156.3 | 13% | 0.018 |
| Hill climbing | 10 127.2 | 13% | 0.049 |
| Simulated annealing | 10 071.2 | 47% | 0.289 |
| **Genetic algorithm** | **31.43 s** | **93%** | – |
| A\* reference | 29.40 s | – | 10 658 expansions |

**Interpretation.** A collision ends the episode, so in a failing route every waypoint after the
collision point sits on a plateau. Local search (one gene or one block at a time) must repair the route
from the first problem onwards and often runs out of budget in the infeasible region. The GA recombines
feasible prefixes and suffixes from different individuals and samples many regions at once. Its median is
within 0.5% (obstacle course) and 7% (warehouse) of the executed A\* route. Some single SA and GA runs beat
the A\* route on the obstacle course, because the grid-optimal path is not time-optimal once the controller's
turning dynamics are included.

## Experiment C: automated failure discovery (Phase 1 subset)

**Question.** Do search-based test generators discover controller failures more efficiently than random
testing? Fitness is collision robustness (guiding property). Success means at least one failing scenario
was found within 2000 scenarios.

### BUG-2: braking distance omitted (hard)

| Generator | Runs with a failure | Median final robustness | Median scenario index of first failure |
|---|---|---|---|
| Random testing | 2 / 15 (13%) | 0.205 | 804 |
| Stochastic hill climbing | 1 / 15 (7%) | 0.404 | 948 |
| Simulated annealing | 0 / 15 | 0.539 | – |
| **Genetic search** | **12 / 15 (80%)** | **−0.048** | 838 |

Genetic search vs random testing: p(Holm) = 0.002, A12 = 0.14 (large). This result is also checked by the
opt-in test `gradlew benchmarkTest` (`SearchVersusRandomBenchmarkTest`).

### BUG-1: lost scan treated as free space (easy)

Every generator except SA finds BUG-1 in all 15 runs. Random testing and genetic search need a median of
**5** scenarios. Search still pays off in *severity*: genetic search finds deeper collisions (median
robustness −0.175 vs −0.135, A12 = 0.19, p(Holm) = 0.018).

**Interpretation.** For dense failures, random testing is as good as anything. BUG-2 is masked by the
controller's conservative heading selection, so only specific combinations of a low free-distance
configuration, high speed and obstacle placement trigger it. The robustness gradient leads the GA towards
those combinations, while random sampling rarely hits them. SA and hill climbing, which follow a single
trajectory, get stuck in the same regions they do in navigation.

## Not yet run

Experiments B (controller tuning), D (multi-objective), E (GP) and F (repair) depend on Phase 2–4 features.
Experiment C will be extended with novelty search, minimisation statistics and coverage-guided generation
in Phase 3.
