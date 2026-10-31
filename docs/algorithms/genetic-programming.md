# Genetic programming of controllers (`controller-synthesis`)

**Intuition.** Instead of tuning the numbers of a hand-written controller, evolve the controller itself: a
small program that maps sensor readings to motion commands (Koza, 1992). Programs are trees; crossover swaps
subtrees, mutation regrows or tweaks them, and fitness comes from simulated driving.

## The language (strongly typed, Montana 1995)

```
ACTION  forward | turnLeft | turnRight | veerLeft | veerRight | stop | if(BOOL, ACTION, ACTION)
BOOL    obstacleAhead | goalLeft | goalAhead | NUM < NUM | NUM > NUM | and | or | not
NUM     front | left | right | goalBearing | goalDistance | constant | NUM + NUM | NUM - NUM | NUM * NUM
```

- Sensor terminals are the minimum range in a sector (front |angle| ≤ 30°, left 30–120°, right −120 to −30°),
  in metres; a lost scan keeps the last valid values. `goalBearing` is relative to the heading (radians).
  `obstacleAhead` = front < 0.7 m, `goalAhead` = |bearing| < 0.25 rad.
- Actions are fixed commands within the robot's limits: forward = (0.7 v_max, 0), turns = (0, ±0.8 ω_max),
  veers = half of both. A program therefore can never command an invalid velocity.
- Every node has a return type and typed arguments; trees are well-typed by construction and the parser
  rejects anything else.

## Safety of execution

| Limit | Mechanism |
|---|---|
| Program size and depth | `maxSize` (40 nodes) and `maxDepth` (7); every operator discards offspring that exceed them |
| Execution per control tick | no loops in the language; the interpreter additionally counts visited nodes and returns `stop` after `stepBudget` (500) |
| Invalid programs | `Program.parse` rejects unknown symbols, wrong argument types or counts, a non-action root, oversize or overdeep trees, with a message |
| Episode length | the simulator's step limit; deadlock detection ends programs that spin in place |

## Operators

| Operator | Definition |
|---|---|
| random | ramped half-and-half: depth uniform in [2, 4], half full trees, half grow trees |
| `subtree` mutation | a random node is replaced by a new grown subtree of the same type (depth ≤ 4) |
| `point` mutation | a node is replaced by another primitive with the same signature, or a constant moves by N(0, 0.5²) |
| `constant` mutation | every constant moves by N(0, 0.3²) |
| `mixed` (default) | subtree 50%, point 35%, constant 15%; a mutation never returns a copy |
| `subtree` crossover | swaps two subtrees of the same type; internal nodes chosen with probability 0.9 (Koza) |
| neighbourhood | all point mutations plus constant steps of ±0.25 and ±1 (hill climbing, SA, tabu work on programs) |
| distance | normalised Levenshtein distance between preorder token lists |

## Fitness and generalisation

Per episode the cost is the travel time on success, otherwise `1000 + 10 × remaining geodesic distance`
(collision, timeout and deadlock all end the episode). Fitness is the mean over the training maps plus
**parsimony pressure** `parsimony × size` (0.05 per node), the bloat control besides the hard limits.
Objectives for NSGA-II: (mean cost, size).

The search only sees the **training maps**. The result reports the best program on separate **test maps**
(`bestExtras.testFitness`, `testSuccessRate`), and the hand-written reactive controller on both sets as the
conventional reference. Maps can be builtin, files, or procedurally generated: `generated:scatter:<seed>`
(20 × 20 m, 15 obstacles, solvability checked), so training and test sets of any size are reproducible.

## Results

Experiment E is in [Phase 4 results](../experiments/phase4-results.md).

## Limitations

- Fixed motion primitives: the program cannot choose speeds continuously.
- Purely reactive: no memory, so U-shaped traps defeat it as they defeat the hand-written controller.
- The test maps measure generalisation to *these* maps; they are not a proof of general competence.
