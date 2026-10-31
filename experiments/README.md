# experiments/

Default output directory of the UI and of `robosbse-engine run|benchmark --out experiments/...`.

- `results/<timestamp>-<name>-<algorithm>-s<seed>/`: `result.json` (self-contained: the configuration
  with the map inlined, best solution, convergence trace, series, archive and replay), `config.json`,
  `convergence.csv`, and `failures/failure-N.json` for testing runs.
- `benchmarks/<timestamp>-benchmark-<name>/`: `benchmark.json`, `runs.csv`, `summary.csv`,
  `pairwise.csv`, `convergence.csv`.
- `configs/`: experiment configurations saved from the UI.

Everything except this file is ignored by git, because results can be regenerated from their
configurations.
