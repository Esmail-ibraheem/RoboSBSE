# Reference planners: A\* and Dijkstra

These are **conventional baselines, not metaheuristics**. They answer the question "what does a classical
planner achieve on this map?", which lets search results be judged in absolute terms.

- **Configuration space:** an `OccupancyGrid` with 0.1 m cells. A cell is blocked when its centre is
  closer than `robot radius + 0.1 m` to an obstacle or the border. The same grid stores the clearance of
  every cell, so it doubles as a sampled distance field.
- **Graph:** 8-connected, with straight steps of `res` and diagonal steps of `√2·res`. A diagonal is only
  allowed when both adjacent orthogonal cells are free (no corner cutting, tested).
- **A\*:** octile heuristic `h = (max(dx,dy) + (√2−1)·min(dx,dy))·res`, which is admissible and
  consistent, so A\* is optimal on the grid. Ties are broken towards larger `g`, deterministically.
- **Dijkstra:** the same search with `h = 0`. It finds the same optimal length but expands more nodes
  (tested on every built-in map).
- **Smoothing:** greedy line-of-sight shortcutting in the inflated grid turns the cell path into a few waypoints.
- **Execution:** the smoothed route is driven by the *same* controller and scored by the *same* fitness
  function as the metaheuristics' routes. That makes the qualities directly comparable.
- **Effort:** reported as **node expansions and microseconds, never as fitness evaluations**. The two kinds of
  effort are not interchangeable, and the benchmark tables say so.

The grid optimum is not the continuous optimum: grid paths are longer than true shortest paths, and
smoothing and controller dynamics change the executed time. So a metaheuristic can beat the "reference"
route. This happened in the obstacle-course benchmark: the best SA and GA runs reached 18.88 s and 18.90 s,
against 19.5 s for the A\* route.

`DistanceField` runs Dijkstra from the goal over the whole grid. It provides the geodesic progress term of
the navigation fitness, and the reachability check that decides whether the liveness property applies to a
test scenario.
