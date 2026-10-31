# Baselines: random search and random walk

## Random search (`random-search`)

**Intuition.** Sample independent, uniformly random solutions and keep the best. Without feedback, its
expected performance after `k` samples is the best of `k` draws from the fitness distribution of the
search space. Any metaheuristic has to beat this to justify its extra machinery (Arcuri and Briand
recommend always reporting it).

- **Representation:** any `SolutionSpace` (`random`).
- **Neighbourhood / population:** none.
- **Fitness evaluation:** one evaluation per sample.
- **Parameters:** none.
- **Stopping:** budget, time limit, target or cancel (through the context).
- **Complexity:** `O(B · C_eval)` for budget `B`, plus `O(n)` per sample to generate it.
- **Strengths:** unbiased; trivially parallel; strong when good solutions are dense (BUG-1 is found after
  a median of 5 scenarios) or when the landscape has plateaus without gradient (BUG-3).
- **Limitations:** ignores all structure. It fails on needle-like targets such as BUG-2 (2 of 15 runs) and on
  constrained route optimisation (20% feasible runs on the obstacle course).
- **Supported problems:** all.

## Exhaustive search (`exhaustive-search`)

**Intuition.** Evaluate every solution of a finite space. If the enumeration finishes within the budget,
the best solution is a **proven optimum**, which is the reference that every heuristic result on the same
instance can be measured against.

- **Representation:** enumerable spaces only (permutations: lexicographic next-permutation order).
- **Stopping:** `ALGORITHM_FINISHED` with stat `complete = 1` after the last solution; budget, time or
  cancel otherwise (`complete = 0`).
- **Complexity:** n! evaluations. The 4-order book has 8 stops, so 40 320 schedules take about 0.3 s.
  The 8-order book (16 stops, about 2·10^13) is far out of reach.
- **Use in RoboSBSE:** `warehouse-orders4-exhaustive.json` proves the optimum (137.5 s makespan, no
  tardiness). `WarehouseSchedulingTest` requires GA, tabu search and SA to get within 2% of it on 3 000
  evaluations.

## Random walk (`random-walk`)

**Intuition.** From a random start, always move to a random neighbour, whatever its fitness. It has the
same neighbourhood as hill climbing and SA but no selection pressure, so comparing against it isolates the
contribution of the acceptance rule.

- **Representation:** `neighbourhood.random` (one block perturbed with `σ = stepSize · range`).
- **Parameters:** `stepSize` (default 0.05).
- **Complexity:** `O(B · C_eval)`.
- **Limitations:** a diffusion process; it rarely beats random search on rugged landscapes.
- **Supported problems:** all.
