# Multi-objective search: Pareto dominance, NSGA-II and hypervolume

## Concepts (`core.Pareto`, `core.Hypervolume`, `core.ParetoArchive`)

- **Pareto dominance** (minimisation): `a ≺ b` if `a` is no worse than `b` in every objective and strictly
  better in at least one. Equal vectors do not dominate each other.
- **Constrained domination** (Deb): a feasible solution dominates any infeasible one. Between two infeasible
  solutions the one with less constraint violation wins. Between two feasible ones ordinary dominance applies.
  This is how failed routes, failed tuning episodes and late deliveries (with `hardDeadlines`) are handled
  without collapsing objectives into a penalty.
- **Fast non-dominated sorting** (Deb et al., 2002) partitions a population into fronts F0, F1, … in
  O(M N²). Every front is mutually non-dominated (tested).
- **Crowding distance:** for each objective, the normalised gap between a solution's two neighbours on
  its front. Boundary solutions get +∞. Larger values mean less crowded.
- **Hypervolume:** the volume of objective space dominated by a front and bounded by a reference point.
  It is computed exactly: an O(n log n) sweep in 2-D, and "hypervolume by slicing objectives" for 3-D
  (validated against Monte Carlo in `ParetoTest`). RoboSBSE reports it **normalised** by the volume of the
  box `[0, reference]`, a fraction in [0, 1]. A value is only meaningful together with its reference point,
  which every result records.
- **Pareto archive:** every run of every algorithm on a multi-objective problem keeps an archive of
  non-dominated *feasible* solutions (default capacity 100; overflow removes the most crowded interior
  point; extremes are kept). Single-objective algorithms (for example a weighted-sum GA) and NSGA-II can
  therefore be compared on the same indicator. The archive's hypervolume is also recorded as a series over
  the run.

## Objectives and reference points per problem

| Problem | Objectives (all minimised) | Reference point |
|---|---|---|
| Route optimisation | travel time (s), energy (J), collision risk `exp(-clearance/0.2 m)` | episode limit, battery capacity, 1 (or `referenceTime`, `referenceEnergy`) |
| Controller tuning | mean time, mean energy, mean risk over training maps | episode limit, battery capacity, 1 |
| Warehouse scheduling | makespan (s), total tardiness (s), energy (J) | 1.5 × greedy schedule, +60 s tardiness |

## NSGA-II (`nsga2`)

```
P ← N random individuals (evaluated)
repeat
    offspring: binary tournaments on (rank, then crowding) -> crossover (p_c) -> mutation -> evaluate
    R ← P ∪ offspring  (2N)
    fronts ← non-dominated sort of R (constrained domination)
    P ← fill with whole fronts; truncate the last one by descending crowding distance
```

- **Representation:** any. It uses the representation's crossover and mutation (on permutations: order,
  PMX or cycle crossover with swap, insert or inversion mutation).
- **Parameters:** `populationSize` (40), `crossover`, `crossoverRate` (0.9), `mutation`, `mutationRate`,
  `mutationSigma`.
- **Recorded:** `firstFrontSize`, `feasible` (per population), `populationHypervolume`, and the archive's
  `hypervolume` series.
- **Complexity:** O(M N²) per generation for sorting, plus N evaluations.
- **Validation:** on ZDT1 (convex front, true hypervolume 2/3 for reference (1, 1)) it reaches more than 0.60
  in 8 000 evaluations, and its archive is mutually non-dominated (tested).
- **Requires** a problem with an objective vector. The registry marks it `multiObjective`, and the runner and
  UI refuse it for single-objective problems such as failure discovery.

## Weighted sum vs Pareto

A weighted sum `Σ w_i f_i` turns the problem into a single-objective one. With fixed weights a run aims at
one point of the front, and it can never reach non-convex parts of it. NSGA-II approximates the whole front
in one run. Experiment D compares both on the same 3-objective navigation task: see
[Phase 2 results](../experiments/phase2-results.md).
