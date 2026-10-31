# Experiment report template

Copy this file, fill in every section and keep the generated `benchmark.json`/`result.json` next to it.

## 1. Research question

> One sentence, e.g. "Under a budget of 2000 fitness evaluations, does a GA find collision-free routes on the
> obstacle course more often than simulated annealing?"

Hypotheses (H0 / H1) and the primary metric (final best fitness, success rate, evaluations to first target, ...).

## 2. Subjects

- Problem type and parameters (`problem.params`), controller and its parameters.
- Environment(s): map names or files (attach custom maps). For testing: arena, properties, fitness guide.
- Why these subjects are representative, and which are deliberately easy or hard.

## 3. Algorithms and configurations

| Label | Algorithm | Parameters | How parameters were chosen (defaults / tuning budget / literature) |
|---|---|---|---|

Tuning on the same seeds you report invalidates the comparison; use separate seeds for tuning.

## 4. Protocol

- Budget: `maxEvaluations` = … (identical for all algorithms); time limit: none or ….
- Seeds: `runs` = … starting at `baseSeed` = … (identical seed lists for all algorithms).
- Reference baselines: random search, A\* for navigation (effort reported separately).
- Command: `java -jar robosbse-engine.jar benchmark <file> --out experiments/benchmarks`.
- Engine version and Java version (in `benchmark.json`).

## 5. Results

- Summary table (`summary.csv`): median, IQR, mean, sd, success rate, median evaluations to first target.
- Box plot of final best fitness and median convergence with quartiles (Benchmark Suite page).
- Pairwise tests (`pairwise.csv`): Mann–Whitney U, Holm-corrected p, A12 and its magnitude.

## 6. Discussion

Interpret the effect sizes as well as the p-values. Explain mechanisms (plateaus, deception, penalty
scale). Compare with the reference baseline.

## 7. Threats to validity

- Internal: implementation bugs (point to the tests), parameter tuning, simulation simplifications.
- External: few maps, a 2D kinematic model, injected rather than real faults.
- Construct: does the fitness measure what matters? Is the oracle complete?
- Conclusion: number of seeds, multiple comparisons (Holm), non-normal distributions (rank tests).

## 8. Reproducibility checklist

- [ ] Benchmark / experiment configuration committed
- [ ] `benchmark.json` (contains all runs and the configuration) archived
- [ ] Engine version recorded
- [ ] Custom maps and failure records attached
