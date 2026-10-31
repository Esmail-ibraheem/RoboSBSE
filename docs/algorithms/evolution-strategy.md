# Evolution strategies (`evolution-strategy`)

**Intuition.** Rechenberg's and Schwefel's evolution strategies evolve real vectors with Gaussian
mutation, and they adapt the mutation step size σ while they search. This implementation is a
(μ/ρ +, λ)-ES.

## Generation

1. For each of λ offspring, draw ρ parents without replacement. The child's object variables are the mean
   of the parents' (intermediate recombination). Its step size is their geometric mean.
2. Mutate the step size, then the variables: `x_d ← clamp(x_d + σ · range_d · N(0,1))`.
3. Survivor selection:
   - **comma** keeps the best μ offspring (requires λ ≥ μ; parents die, which helps leave stale σ behind);
   - **plus** keeps the best μ of parents and offspring (elitist, tested).

## Step-size control (`stepControl`)

| Mode | Rule |
|---|---|
| `self-adaptive` (default) | every individual carries σ; `σ' = σ · exp(τ N(0,1))` with `τ = 1/√n`. Good σ values survive with the solutions they produce |
| `one-fifth` | a single global σ. Every `adaptInterval` generations, if more than 1/5 of the offspring beat their best parent, σ /= 0.82, otherwise σ *= 0.82 (Rechenberg's 1/5 success rule) |

- **Parameters:** `mu` (5), `lambda` (35), `rho` (2), `selection` (comma | plus), `initialSigma` (0.1 × range),
  `minSigma` (1e-5), `stepControl`, `adaptInterval` (5).
- **Recorded:** `sigma` (mean step size), `populationBest`, `successRate` (1/5 rule).
- **Complexity:** λ evaluations per generation, plus `O(λ log λ)` for sorting.
- **Strengths:** the step size adapts to the landscape scale automatically. All four combinations solve
  the 5-D sphere to 10⁻³ (tested). In controller tuning it is statistically tied with the GA.
- **Limitations:** real vectors only. One σ per individual cannot handle badly scaled or rotated landscapes;
  that needs CMA-ES, which is not implemented.
- **Supported problems:** real-vector problems.
