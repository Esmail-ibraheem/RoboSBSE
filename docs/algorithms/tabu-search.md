# Tabu search (`tabu-search`)

**Intuition.** Hill climbing stops at the first local optimum. Tabu search (Glover, 1986) keeps moving:
each iteration it takes the **best admissible neighbour, even if that neighbour is worse**. A short-term
memory stops it from simply undoing that step. After a move, the attribute of the move that would reverse
it becomes *tabu* for `tenure` iterations.

## Moves and memory

`Neighbourhood.moves(s)` lists the neighbourhood as `(result, attribute, inverse)` triples:

| Representation | Neighbourhood | Attribute / inverse |
|---|---|---|
| real vector | one gene ± step | `"3+"` / `"3-"`: after raising gene 3, lowering it again is tabu |
| permutation | `swap` | the unordered pair of swapped *elements* (`swap:2,7`); the inverse is the same pair |
| permutation | `insert` | the moved element (`move:5`); moving it again is tabu |
| permutation | `inversion` (2-opt) | the segment ends |
| any other | default | solution-based: the neighbour itself; the inverse is the current solution (revisiting is tabu) |

## Selection rule (`TabuSearch.select`, unit-tested)

1. Evaluate all moves, or a random **candidate list** of `candidates` moves when the neighbourhood is large.
2. A move is admissible if it is not tabu, **or** it is tabu but would produce a new best-so-far solution
   (**aspiration**, `aspiration = true`).
3. Take the admissible move with the lowest fitness, worse or not.
4. If no move is admissible, the move whose tabu status expires first is forced.
5. Make the inverse tabu until `iteration + tenure`. Optionally restart from a random solution after
   `maxStagnation` iterations without a new best.

- **Parameters:** `tenure` (7), `candidates` (0 = full neighbourhood), `aspiration` (true),
  `maxStagnation` (0 = never), `stepSize` (real vectors), `neighbourhood` (permutations).
- **Stopping:** budget, time, target or cancel.
- **Complexity:** each iteration costs |neighbourhood| evaluations (or `candidates`). The tabu list
  lookup is O(1) (hash map).
- **Recorded:** `current` (fitness of the accepted solution each iteration, which also goes *up*), `tabuSize`,
  `aspirations`, `forcedMoves`, `restarts`.
- **Strengths:** deterministic escape from local optima, well suited to permutation problems.
  `Phase2AlgorithmsTest` checks that it reaches the proven optimum of a 9-city circle tour.
- **Limitations:** a full neighbourhood is expensive. With 16 stops the swap neighbourhood has 120 moves,
  so a 4 000-evaluation budget allows only about 33 iterations. In a first scheduling benchmark,
  full-neighbourhood swap tabu had a median of 435 s against 322 s for SA. With a random candidate list of
  20 moves, its median became 322 s, the best known (Phase 2 results).
- **Supported problems:** all.
