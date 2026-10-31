# Genetic algorithm (`genetic-algorithm`)

**Intuition.** Evolve a population. Selection prefers fitter individuals, crossover combines building
blocks of two parents (for routes: good route prefixes and suffixes), and mutation keeps exploring. The
population samples many regions at once, which helps on deceptive, plateau-rich landscapes such as
collision-terminated routes and needle-like failure conditions.

## Loop

```
P ← populationSize random individuals (evaluated)
repeat
    offspring ← ∅
    while |offspring| < needed:
        p1, p2 ← select(P), select(P)
        children ← crossover(p1, p2) with probability crossoverRate, else copies
        for each child: offspring += evaluate(mutate(child))
    P ← replace(P, offspring)
    record best / mean / worst fitness and diversity
```

- **Selection** (`selection`):
  - `tournament`: best of `tournamentSize` uniform draws with replacement.
  - `roulette`: fitness-proportional selection, made valid for minimisation and negative fitness by
    *windowing*, `w_i = (f_worst − f_i) + 0.01·(f_worst − f_best)`. Plain roulette would be invalid here.
  - `rank`: linear ranking with pressure `s ∈ [1, 2]`. The best individual has probability `s/N`, the worst `(2−s)/N`.
- **Crossover / mutation:** the `RealVectorSpace` operators (see the [index](README.md)). Children are always
  mutated with per-gene rate `mutationRate` (0 ⇒ `1/n`) and `σ = mutationSigma · range`.
- **Replacement** (`replacement`):
  - `generational`: the `elitism` best parents survive unchanged and are not re-evaluated; the remaining
    `N − e` places are filled with offspring.
  - `plus`: (μ+λ). Parents and `N` offspring compete and the best `N` survive, which is implicitly elitist.
  - `steady-state` (also registered as the separate algorithm `steady-state-ga`): offspring are produced
    two at a time, and each child replaces the current worst individual if it is better. The population
    best therefore never gets worse (tested), and new genetic material is available to selection
    immediately. One "generation" for statistics is `populationSize` offspring.
- **Diversity:** mean pairwise normalised genotypic distance, `O(N²·n)` per generation.
- **Parameters:** `populationSize` (50), `selection`, `tournamentSize` (3), `rankPressure` (1.5),
  `crossover`, `crossoverRate` (0.8), `mutation`, `mutationRate` (0 = 1/n), `mutationSigma` (0.1),
  `elitism` (2), `replacement`.
- **Stopping:** budget, time, target or cancel. If the budget runs out in mid-generation, the generation
  is cut short, and the best individual seen so far is still reported by the context.
- **Complexity:** `O(B · C_eval)` evaluations, plus `O(N²·n)` per generation for the diversity measure
  (negligible next to simulation costs here).
- **Strengths:** clearly the strongest Phase 1 algorithm on both problems. It found a feasible route in 93%
  of runs (median within 1% of the A\* route's time) and BUG-2 in 80% of runs, against 13% for random testing.
- **Limitations:** more parameters. Premature convergence shows up as falling diversity. Population
  initialisation spends `N` evaluations before any selection happens.
- **Supported problems:** every representation that supplies crossover and mutation: real vectors
  (including the random-key encodings of the arm and multi-robot problems), permutations (scheduling),
  programs (genetic programming) and patches (repair, improvement).
