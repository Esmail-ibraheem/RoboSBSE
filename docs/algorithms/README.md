# Algorithms

Every algorithm implements `SearchAlgorithm.run(SearchContext)`. It asks the context for evaluations and is
stopped by the context (budget, time limit, target, cancel). Fitness is **minimised**. Each algorithm is
written once, generically in the solution type `S`, and uses only the operators that the problem's
`SolutionSpace<S>` provides:

| Algorithm | Needs | Page |
|---|---|---|
| Random search | `random` | [baselines.md](baselines.md) |
| Random walk | `neighbourhood.random` | [baselines.md](baselines.md) |
| Hill climbing (first / best / stochastic, restarts) | `neighbourhood.all` / `.random` | [hill-climbing.md](hill-climbing.md) |
| Simulated annealing | `neighbourhood.random` | [simulated-annealing.md](simulated-annealing.md) |
| Exhaustive search | `enumerate` (permutations) | [baselines.md](baselines.md) |
| Tabu search | `neighbourhood.moves` (move attributes) | [tabu-search.md](tabu-search.md) |
| Genetic algorithm (generational, (mu+lambda), steady-state) | `random`, `mutation`, `crossover`, `distance` | [genetic-algorithm.md](genetic-algorithm.md) |
| Evolution strategy | real vectors | [evolution-strategy.md](evolution-strategy.md) |
| Particle swarm optimisation | real vectors | [particle-swarm.md](particle-swarm.md) |
| NSGA-II, Pareto archive, hypervolume | objective vector | [nsga2.md](nsga2.md) |
| Memetic algorithm (GA + hill climbing / SA / route smoothing) | GA operators + a neighbourhood | [memetic.md](memetic.md) |
| Novelty search, behaviour coverage | GA operators + behaviour descriptors | [novelty-search.md](novelty-search.md) |
| Island-model GA, parallel fitness evaluation | GA operators | [island-model.md](island-model.md) |
| Genetic programming (typed controller programs) | `program` representation: ramped half-and-half, subtree crossover, subtree/point/constant mutation | [genetic-programming.md](genetic-programming.md) |
| Patch search (program repair, genetic improvement) | `patch` representation: edits at JavaParser mutation points | [../research/program-repair.md](../research/program-repair.md) |
| A\* / Dijkstra (reference planners, not metaheuristics) | occupancy grid | [reference-planners.md](reference-planners.md) |

Population-based algorithms (GA, NSGA-II, ES, synchronous PSO, random search, memetic, novelty, islands)
hand whole generations to `SearchContext.evaluateAll`, which evaluates them in parallel when the run has
several evaluation threads and still produces bit-identical results for every thread count
([island-model.md](island-model.md)). Coverage-guided test generation is described in
[../research/coverage.md](../research/coverage.md).

The UI and the runner check compatibility: each algorithm declares the representations it supports
(`any`, `real-vector`, `permutation`) and whether it needs objectives, and incompatible combinations are
rejected with an explicit message instead of failing silently.

## The shared representation: bounded real vectors

Both Phase 1 problems encode candidates as `RealVector`s inside a box `[l_i, u_i]` (`RealVectorSpace`).
Genes can be grouped into **blocks** (block size 2 for route waypoints), and the operators respect them:

| Operator | Definition |
|---|---|
| random | `x_i ~ U(l_i, u_i)` |
| `gaussian` mutation | each gene with probability `p` (default `1/n`): `x_i += N(0, (σ·(u_i−l_i))²)`, clamped. If no gene was picked, one random gene is mutated, so a mutation never returns a copy |
| `uniform-reset` mutation | each gene with probability `p`: `x_i ~ U(l_i, u_i)` (same at-least-one rule) |
| `one-point` / `two-point` crossover | cut points at block boundaries, tails/middles swapped |
| `uniform` crossover | each block swapped with probability 1/2 |
| `arithmetic` crossover | `c1 = αa + (1−α)b`, `c2 = (1−α)a + αb`, `α ~ U(0,1)` |
| enumerable neighbourhood | `x ± step·(u_i−l_i)` for one gene (≤ 2n neighbours); moves that cannot leave the bound are dropped |
| random neighbour | Gaussian perturbation of every gene of one random block, `σ = step·range` |
| distance | `sqrt(mean_i ((a_i − b_i)/(u_i − l_i))²)` ∈ [0, 1] (used for diversity) |

The real-vector representation suits route optimisation, controller tuning and scenario generation,
because those decision spaces are continuous and box-shaped. Applying continuous PSO to permutations
without a defined encoding is explicitly refused.

## The permutation representation (`PermutationSpace`)

Used by warehouse task scheduling, where the permutation is a priority list:

| Operator | Definition |
|---|---|
| random | Fisher-Yates shuffle |
| mutation `swap` / `insert` / `inversion` / `scramble` | exchange two elements / move one element / reverse a segment (2-opt) / shuffle a segment; one move per mutation, never a copy |
| crossover `order` (OX) | keep a segment of parent A, fill the rest in parent B's order starting after the segment |
| crossover `pmx` | keep a segment of A, place B's genes through the segment's mapping |
| crossover `cycle` (CX) | cycles of positions alternate between the parents; every position comes from one parent (tested) |
| neighbourhood `swap` / `insert` / `inversion` | all n(n-1)/2 swaps / (n-1)^2 distinct insertions / n(n-1)/2 segment reversals, with move attributes for tabu search |
| distance | normalised Hamming distance (fraction of positions that differ) |
| enumerate | all n! permutations in lexicographic order (exhaustive search) |

Operator parameters accept `"default"`, which means the representation's first operator. The UI only offers
the operators of the selected problem's representation.

## Invariants under test

`LocalSearchTest`, `GeneticAlgorithmTest`, `SearchContextTest` and `RealVectorSpaceTest` check:

- convergence on problems with known optima (the sphere; a circle tour whose optimum is the polygon;
  ZDT1 for NSGA-II; the exhaustive optimum of a 4-order schedule for GA, tabu and SA);
- tabu memory: tabu moves are rejected unless aspiration applies, and the soonest-expiring move is forced
  when everything is tabu;
- PSO with both topologies, and all four ES variants, on the sphere; plus-selection ES and the steady-state
  GA never lose their best;
- exact hypervolume against known values and Monte Carlo; non-dominated sorting and crowding distance;
  archive truncation keeps the extremes;
- that first- and best-improvement hill climbing only accept strictly better moves, and that best
  improvement evaluates the whole neighbourhood;
- the Metropolis rule, exactly and by frequency (`P = e^-1` within ±0.005 for Δ = T);
- that cooling schedules decrease and that budget-derived cooling ends at `T0·10⁻³`;
- that elitism never loses the population best, and that without elitism the best *can* be lost;
- a constant population size, and that (μ+λ) keeps the best of parents and offspring;
- selection pressure: tournament size 1 is uniform, roulette favours better individuals under
  minimisation, rank selection gives the best individual probability `s/N`;
- exact budgets, determinism for a fixed seed, pause, cancel and time limit;
- that operators stay inside the bounds, that crossover never splits a block, and that the arithmetic
  child lies between its parents.
