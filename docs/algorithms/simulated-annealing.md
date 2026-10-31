# Simulated annealing (`simulated-annealing`)

**Intuition.** A local search that sometimes accepts worse moves, so it can escape local optima. The
probability of accepting a worsening `Δ > 0` at temperature `T` follows the **Metropolis rule**

```
P(accept) = 1                 if Δ ≤ 0
P(accept) = exp(−Δ / T)       otherwise
```

Early, at high `T`, the search explores almost freely. As `T → 0` it behaves like stochastic hill climbing.
With logarithmic cooling the process converges in probability to a global optimum (Geman and Geman), but
too slowly for practical budgets. Faster schedules give up that guarantee in exchange for speed.

## Implementation

- **Neighbourhood:** random neighbour (one block, `σ = stepSize · range`).
- **Acceptance:** `SimulatedAnnealing.accept(Δ, T, u)` with `u ~ U(0,1)`. The random number is drawn on
  every move whether or not it is needed, so runs stay reproducible. The rule is tested exactly and
  by frequency.
- **Cooling schedules** (`cooling`), applied every `stepsPerTemperature` moves (the Markov-chain length):

| name | update |
|---|---|
| `geometric` | `T ← α·T`; with `alpha = 0`, `α = finalTemperatureRatio^(1/K)` so that `T` reaches `T0 · finalTemperatureRatio` exactly at the end of the budget (`K` steps) |
| `linear` | `T_k = T0 · (1 − k/K)` |
| `logarithmic` | `T_k = T0 / ln(e + k)` |
| `lundy-mees` | `T ← T / (1 + β·T)` |

- **Initial temperature:** `initialTemperature > 0` sets `T0` directly. With 0 (the default), `T0` is
  **calibrated**: `calibrationSamples` random neighbours of the start solution are evaluated (charged to the
  budget), and `T0 = −median(Δ⁺) / ln(initialAcceptance)`, so the median worsening move is accepted with
  probability `initialAcceptance` (0.8).
- **Recorded:** `temperature` and `current` series, plus `acceptanceRate`, `worseAccepted`,
  `initialTemperature` and `alpha` stats.
- **Parameters:** `initialTemperature`, `initialAcceptance`, `cooling`, `alpha`, `finalTemperatureRatio`,
  `beta`, `minTemperature`, `stepsPerTemperature`, `calibrationSamples`, `stepSize`.
- **Stopping:** budget, time, target or cancel. SA never stops by itself.
- **Complexity:** `O(B · C_eval)`, with `O(1)` overhead per move.
- **Strengths:** escapes shallow local optima; needs only a random neighbour.
- **Limitations:** sensitive to scale. The navigation fitness mixes successful routes (≈20–40 s) with
  penalised failures (≥10⁴), so a temperature tuned to one regime is wrong for the other. That is the
  reason for the calibration and the budget-relative cooling. A single trajectory without restarts can
  spend its budget in a bad region (40% feasible runs on the obstacle course, 0/15 on BUG-2).
- **Supported problems:** all `RealVectorSpace` problems.
