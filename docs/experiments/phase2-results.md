# Phase 2 results

Measured on 2026-10-27 (engine 0.1.0, Java 21.0.12, Windows 10, 8 worker threads). Every benchmark used 15
seeds per algorithm and an identical evaluation budget for all algorithms. Configurations are in
`examples/benchmarks/`. To reproduce:

```bash
java -jar backend/java-engine/build/libs/robosbse-engine.jar benchmark examples/benchmarks/<file>.json --out experiments/benchmarks
```

## Experiment B: controller parameter tuning across environments

**Question.** Which optimisation approach finds effective parameters for the map-free reactive controller
across several environments?

**Setup.** Five parameters were tuned: speed limit, safety margin, heading gain, free-distance threshold and
approach gain. Fitness is the mean over three training maps (obstacle course, warehouse, empty field) of
`travel time + 20 × collision risk`. A failed episode costs 1000 plus 10 × the remaining geodesic distance.
The testing arena is a **held-out validation map**, never used during search. Budget: 600 evaluations,
each one three full simulated episodes.

| Algorithm | Median training fitness | All training maps solved | Median validation fitness |
|---|---|---|---|
| Untuned defaults (reference) | 391.3 (fails the warehouse) | no | 14.20 |
| Random search | 32.88 | 15/15 runs | 13.40 |
| Hill climbing (stochastic) | 31.22 | 15/15 | 12.95 |
| Genetic algorithm (pop 20) | 31.04 | 15/15 | 12.95 |
| Evolution strategy (5/2, 35) | 31.08 | 15/15 | 13.00 |
| **PSO (20 particles)** | **30.76** | 15/15 | 12.95 |
| Simulated annealing | 50.68 | 8/15 | 14.31 |

PSO vs random search: p(Holm) = 6·10⁻⁵, A12 = 0.996. GA vs random search: p(Holm) = 1·10⁻⁴, A12 = 0.982.
PSO vs SA: p(Holm) = 0.011, A12 = 0.858. GA vs PSO and ES vs PSO are not significant after Holm correction.

**Interpretation.** The untuned controller cannot complete the warehouse map with its defaults, while every
population-based tuner produces parameters that solve all three training maps. The improvement carries
over to the unseen arena (12.95 vs 14.20). That is evidence that the tuned parameters generalise beyond the
maps they were tuned on, at least to this one held-out map. SA, a single trajectory with a temperature
calibrated on a landscape mixing failure penalties (≥1000) with successes (≈30), often ends in parameters
that fail a map. Random search keeps the largest hypervolume of (time, energy, risk) trade-offs (0.505 vs
0.47 for PSO), because it never concentrates on one region; the guided methods trade diversity for a better
scalar optimum.

An easier variant shows why the setup matters. With the PID heading controller and a pure travel-time
objective (`tuning-pid-pso.json`), every method converges to the same 23.88 s, because the optimum is simply
the speed limit. A benchmark there does not discriminate between algorithms.

## Experiment D: weighted sum vs Pareto optimisation

**Question.** How does optimising travel time, energy and collision risk jointly change the result?

**Setup.** Obstacle course, 4 waypoints, 3 000 evaluations. The weighted-sum GA minimises
`time + 0.1·energy + 20·risk`; NSGA-II optimises the three objectives directly. Both are compared on the
normalised hypervolume of their run's archive of non-dominated feasible routes. Reference point: 60 s,
800 J, risk 1. Random search is the baseline.

| Algorithm | Median hypervolume | Median weighted fitness | Runs with a feasible route |
|---|---|---|---|
| Weighted-sum GA | 0.454 | 46.05 | 13/15 |
| NSGA-II | 0.457 | 46.25 | 14/15 |
| Random search | 0.000 | 10 093.7 | 5/15 |

Weighted-sum GA vs NSGA-II on hypervolume: p = 0.74, A12 = 0.46 (negligible). Both beat random search
(p(Holm) = 2·10⁻⁴, A12 = 0.92).

**Interpretation.** On this problem the two approaches end up equivalent, by both indicators. The
weighted-sum GA's archive collects many non-dominated routes along the way, and the front is narrow: once
the route is collision-free, time and energy move together because both grow with path length. Only risk
pulls the other way. NSGA-II found a feasible route slightly more often and gives the whole trade-off curve
in one run; the weighted sum gives one compromise per weight vector. A wider front, for example with the
smoothness or tardiness objectives, would be needed to separate them. This is a null result, reported as one.

## Warehouse task scheduling (orders-8)

**Setup.** 8 pickup-and-delivery orders (16 stops), capacity 2, three dependencies, deadlines, 35% initial
battery (charging needed). Objective: `makespan + 5 × tardiness`. Budget 4 000 evaluations.

| Algorithm | Median fitness | All deadlines met | Best |
|---|---|---|---|
| Greedy dispatching (reference) | 396.2 (one late order) | no | – |
| Random search | 734.3 | 0/15 | 461.1 |
| Hill climbing (first improvement, swap) | 348.1 | 15/15 | 322.2 |
| **Simulated annealing (insert)** | **322.2** | 15/15 | 322.2 |
| **Tabu search (swap, 20-move candidate list)** | **322.2** | 12/15 | 322.2 |
| Genetic algorithm (order crossover, inversion) | 363.3 | 10/15 | 322.2 |
| Steady-state GA | 361.3 | 8/15 | 322.2 |

SA vs GA: p(Holm) = 8·10⁻⁴, A12 = 0.91. SA vs tabu: not significant. Every method beats random search
(p(Holm) ≤ 0.004).

**Interpretation.** On this priority-list representation, local search with good moves (SA on insertions,
tabu on swaps with a candidate list) beats the GA. The decoder makes neighbouring permutations map to
similar schedules, so small moves are effective. The value 322.2 was found by every method at least once
and is the best known; it is not proven optimal (16! orders cannot be enumerated). On the 4-order book,
exhaustive search proves the optimum (137.5) and GA, tabu and SA reach it within 2% (tested). The greedy
dispatcher misses a deadline (13.7 s late) with a 327.8 s makespan. The best-known schedule is better on
both counts: 322.2 s and no lateness. Individual GA runs that meet every deadline do so at the price of a
longer makespan (for example 356 s), which the weighted objective accepts.

**Model check.** Replays drive the decoded mission through the simulator leg by leg. For the GA's best
schedule, simulated travel time was 242.4 s against 246.9 s from the distance/speed model (−1.8%).
`WarehouseSchedulingTest` requires agreement within 25%.
