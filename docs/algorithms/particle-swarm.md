# Particle swarm optimisation (`particle-swarm`)

**Intuition.** A swarm of particles flies through a continuous search space. Each particle is pulled
towards the best point *it* has seen (cognitive term) and the best point its *neighbourhood* has seen
(social term), and keeps part of its previous velocity (inertia) (Kennedy and Eberhart, 1995).

```
v_d = w v_d + c1 r1 (p_d - x_d) + c2 r2 (g_d - x_d)      r1, r2 ~ U(0,1) per dimension
|v_d| <= vMax * range_d
x_d = x_d + v_d          (clamped to the box; the velocity component is zeroed at a bound)
```

- **Representation:** real vectors only. PSO is not applied to permutations, because a permutation has no
  defined continuous encoding here; the registry and the runner reject that combination with an explicit
  message.
- **Topology:** `global` (gbest) or `ring` (lbest: the particle and its two index neighbours).
- **Updates:** the guide of every particle is chosen at the start of an iteration. With `update = immediate`
  (default, used for all Phase 2 results) particles move and are evaluated one after another, and a personal
  best is replaced as soon as the particle improves on it, so a particle whose guide improved earlier in the
  same iteration follows the guide's new position. With `update = synchronous` every particle moves on the
  iteration-start bests and the swarm is evaluated as one (parallelisable) batch. Both draw the same random
  numbers. (Before Phase 3 this page wrongly described the default as synchronous.)
- **Defaults:** Clerc and Kennedy's constriction-equivalent constants: `inertia` 0.7298,
  `cognitive = social` 1.49618, `maxVelocity` 0.2 × range. `inertiaEnd ≥ 0` gives a linearly decreasing
  inertia over the evaluation budget.
- **Recorded:** `swarmDiversity` (mean normalised distance to the swarm centroid), `globalBest`, `inertia`.
- **Complexity:** `swarmSize` evaluations per iteration, plus `O(swarmSize · n)` arithmetic.
- **Strengths:** few parameters and strong on smooth continuous landscapes. It was the best method in
  Experiment B (controller tuning: median 30.76 vs 32.88 for random search, A12 = 0.996).
- **Limitations:** it can converge prematurely (diversity collapses), and a box constraint is handled by
  clamping, which piles particles up on bounds when the optimum lies there.
- **Supported problems:** route optimisation (waypoint coordinates), controller tuning, failure discovery
  (scenario vectors).
