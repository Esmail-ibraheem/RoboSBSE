# Memetic algorithm (`memetic-algorithm`)

**Intuition.** A genetic algorithm explores broadly but refines slowly. A local search refines quickly but
gets stuck. A memetic algorithm combines them: the GA breeds offspring as usual, and some offspring are
improved by a short local search before they compete for survival (Moscato, 1989).

```
population = evaluate(random individuals);  refine each with probability localRate
repeat
    offspring = evaluate(mutate(crossover(select(population))))      one batch, parallelisable
    for each child, with probability localRate:
        child = localSearch(child, localBudget evaluations)
    population = replace(population, offspring)                     generational + elitism, or (mu+lambda)
```

- **Local searches** (`localSearch`), each limited to `localBudget` evaluations:
  - `hill-climbing`: stochastic first improvement in the representation's neighbourhood (real vectors: one
    block perturbed with σ = `localStepSize` × range; permutations: the `neighbourhood` moves);
  - `simulated-annealing`: Metropolis acceptance with a temperature that falls linearly from
    `T0 = localTemperature · max(1, |f(start)|)` to 0 over the local budget; the best solution visited is
    returned;
  - `problem`: stochastic first improvement in the problem's own neighbourhood
    (`Problem.domainNeighbourhood()`). Navigation provides **route smoothing**: a move pulls one waypoint the
    fraction α ~ U(0.25, 1) of the way to the midpoint of its neighbours on the route (start, waypoints, goal).
    Other problems have none, and the algorithm refuses `problem` for them with an explicit message.
- **Learning** (`learning`): `lamarckian` writes the improved solution back into the population;
  `baldwinian` keeps the original genotype but gives it the improved fitness, so selection favours
  individuals that are *easy to improve* without copying the improvement (Hinton and Nowlan, 1987).
- **Budget accounting:** every local-search evaluation goes through the shared `SearchContext` and is
  charged to the run's budget. A memetic algorithm and a GA with the same budget are therefore directly
  comparable, and the run's best solution includes everything a local search found.
- **Recorded:** the GA's population series plus `localSuccessRate` (share of local searches that improved),
  `localMeanGain` (mean fitness gain of successful searches), and the stats `localSearches`,
  `localEvaluations`, `localShare` (share of the budget spent in local search). With `localRate = 0.5` and
  `localBudget = 10` the share is 5/6, which a test checks.
- **Complexity:** per generation `offspring · (1 + localRate · localBudget)` evaluations on average.
- **Strengths:** reliable final refinement. On the obstacle course the hill-climbing and route-smoothing
  variants reached the goal in 15 of 15 runs, the plain GA in 13 of 15 ([Phase 3 results](../experiments/phase3-results.md)).
- **Limitations:** local search costs evaluations the GA could have spent exploring. The median best fitness
  was not significantly different from the GA's in that experiment. Steady-state replacement is not
  supported.
- **Supported problems:** all (local search needs a neighbourhood; `problem` needs a domain neighbourhood).
