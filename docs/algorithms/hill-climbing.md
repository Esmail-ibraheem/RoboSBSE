# Hill climbing (`hill-climbing`)

**Intuition.** Greedy local search: start somewhere and repeatedly move to a better neighbour. The search
stops at a **local optimum**, a solution with no better neighbour. Random restarts turn this into
*random-restart hill climbing*: the overall result is the best local optimum found within the budget.

## Variants

| `variant` | Move rule | Cost per move |
|---|---|---|
| `first` | scan the enumerable neighbourhood **in random order**, move to the first strictly better neighbour | 1 … 2n evaluations |
| `best` (steepest ascent) | evaluate the whole neighbourhood, move to the best neighbour if it is strictly better | exactly 2n evaluations (tested) |
| `stochastic` | sample random neighbours, move on improvement (or on ties with `acceptEqual`); `maxFailures` consecutive failures ⇒ local optimum (with high probability) | ≥ 1 evaluation |

Randomising the scan order of first improvement avoids a systematic bias towards low gene indices.

- **Representation / neighbourhood:** real vectors; neighbourhood = one gene ± `stepSize · range` (≤ 2n
  neighbours); the random neighbour perturbs one block with `σ = stepSize · range`.
- **Fitness evaluation:** through the context; every neighbour costs one evaluation.
- **Parameters:** `variant`, `stepSize` (0.05), `restarts` (true), `maxFailures` (50, stochastic only),
  `acceptEqual` (false, stochastic only).
- **Stopping:** without restarts the algorithm stops at the first local optimum (`ALGORITHM_FINISHED`),
  which leaves budget unused. With restarts it stops on budget, time, target or cancel. The context records
  `localOptimum` (fitness of each local optimum) and `current` (accepted moves) series.
- **Complexity:** best improvement costs `O(n · C_eval)` per move. First improvement costs the same in the
  worst case and much less while improvements are easy to find.
- **Strengths:** exploits smooth, unimodal regions efficiently, and its behaviour is simple to analyse.
- **Limitations:** trapped by local optima and **plateaus**. In route optimisation a collision ends the
  episode, so moving waypoints *after* the collision point changes nothing. These plateaus explain why
  hill climbing reached a feasible route in only 27% of obstacle-course runs (see the Phase 1 results).
  A fixed step size trades precision against reach.
- **Supported problems:** all `RealVectorSpace` problems (navigation, testing). In principle any
  representation with a finite neighbourhood.
