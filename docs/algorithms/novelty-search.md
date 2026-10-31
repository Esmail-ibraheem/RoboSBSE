# Novelty search (`novelty-search`) and behaviour coverage

**Intuition.** On deceptive problems the fitness gradient leads into dead ends. Novelty search ignores the
objective and rewards individuals that *behave* differently from everything seen so far (Lehman and
Stanley, 2011). Exploring behaviour space systematically can reach solutions that fitness-driven search
walks away from.

## Behaviour descriptors

Every `Evaluation` carries a behaviour descriptor (`Evaluation.behaviour()`), components scaled to roughly [0, 1]:

| Problem | Descriptor |
|---|---|
| navigation | final position, then positions at 75%, 50% and 25% of the episode, each divided by the map size (8 numbers) |
| controller testing | the same 8 path numbers of the controller under test, its closest approach to an obstacle (clamped to [0, 1] m) and the episode duration as a fraction of the step limit |
| controller tuning | per training episode: duration / step limit and minimum clearance (clamped to [0, 1] m) |
| warehouse scheduling | (makespan, tardiness, energy) divided by the hypervolume reference point |

The positions at fixed *fractions* of the episode come from a coarse position history that the simulator
keeps for every episode (`EpisodeResult.pathSamples`). Fractions rather than fixed times are used because
route-following episodes are much shorter than the step limit, so fixed times would mostly repeat the end
point.

## Algorithm

```
novelty(i) = mean distance from b(i) to its k nearest neighbours in  population ∪ offspring ∪ archive
score(i)   = (1 - w) · noveltyRank(i) + w · fitnessRank(i)          ranks in [0, 1], 1 = most novel / fittest
```

Each generation: tournament selection on the score, crossover and mutation, batch evaluation of the
offspring, novelty of parents and offspring, archive insertion of offspring whose novelty exceeds the
threshold (first-in-first-out beyond `maxArchive`), and (μ + λ) truncation on the score. The threshold rises
by 20% after a generation with more than `addsPerGeneration` additions and falls by 5% after five
generations in a row without any. Candidates without a descriptor (invalid test scenarios) have novelty 0.

- **`fitnessWeight` (w):** 0 is pure novelty search. Larger values blend in the objective; rank-based blending
  avoids having to scale novelty and fitness against each other.
- **Recorded:** `meanNovelty`, `archiveSize`, `populationBest` series; `noveltyThreshold` stat. The context
  still tracks the best fitness and every target hit, so novelty search is judged by the same measures as
  any other algorithm.
- **Complexity:** per generation O((P + λ) · (P + λ + A) · d) for the k-nearest-neighbour search, with
  population P, offspring λ, archive A ≤ `maxArchive` and descriptor length d. Negligible next to simulation.

## Behaviour coverage (every run)

Independently of the algorithm, the `SearchContext` marks the cell of a 10 × 10 grid over the first two
descriptor components of every evaluated candidate (for routes and test scenarios: where the robot
*ended*). `behaviourCoverage` is the fraction of visited cells; it is recorded as a series and a stat for
every run of a problem that defines behaviour. It is comparable only within one problem configuration,
because not every cell is reachable (walls, short episodes).

## Results and limits

- Maze (6 000 evaluations, 15 runs): novelty search visited a median 20% of the end-position grid, the GA 7%
  and random search 10%. **No algorithm reached the goal**, so here broader exploration did not translate
  into success within the budget.
- Failure discovery for BUG-2: pure novelty found the defect in 6 of 15 runs, collision-guided GA in 12,
  novelty blended with fitness (w = 0.5) in 14. See [Phase 3 results](../experiments/phase3-results.md).
- The descriptor decides what "new" means. A poor descriptor makes novelty search a random walk in
  behaviour space.
