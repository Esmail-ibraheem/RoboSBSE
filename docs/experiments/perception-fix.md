# Perception fix of the correct reactive controller

**Status (2026-10-31).** The fix is implemented and tested, and its effect on safety is measured below. The
experiments that use the reactive controller family have **not yet been rerun** with it: Experiments B and C,
the BUG-3 coverage campaign, Experiment F (RB1–RB5) and genetic improvement. Both rerun suites were started
and then stopped by the host because the machine ran low on memory. The Phase 1–4 result documents
therefore still describe the controller **before** the fix. Commands for the reruns are at the end.

## The defect

Phase 4 found three scenarios in which the correct `reactive` controller penetrates an obstacle by
0.6–1.4 cm (now in `examples/faulty-controllers/fixed-issues/reactive-sparse-rays-*.json`). Its safety checks
only know the obstacle points hit by its 24 range rays, 15° apart. A surface between two rays, typically a
corner pointing at the robot, can come closer than any sampled point. Scan dropout makes this worse: the
robot keeps moving on points sampled from farther away, which lie further apart.

A direct search confirms that the defect is not rare under search. A collision-guided GA (population 50,
2000 scenarios, `examples/benchmarks/experiment-C-correct-controller.json`) finds a collision of the
original controller in **7 of 20** runs.

## Variants tried

Measured on: the recorded collision scenarios; 337 valid random arena scenarios (seed 2024); navigation from
start to goal on 13 maps (5 built-in, 8 generated); and the GA collision search above. "Corner guard": for
adjacent rays that both hit at most `g · (radius + margin)` apart, add the point where a right-angled corner
joining the two hit points would come closest to the robot (on the circle with the two points as diameter).
"Point memory k": use the obstacle points of the last `k` valid scans (obstacles are static).

Rows marked **R** are reproduced by the committed script (`cornerGuard:pointMemory` variants on the final
code; 20 search seeds):

```bash
java -cp backend/java-engine/build/libs/robosbse-engine.jar scripts/perception/PerceptionVariants.java 20 false:1 true:1 false:3 false:4 false:5 true:4 true:6
```

Rows marked **E** were exploratory: measured during the investigation with temporary code that was not kept
(other guard factors, the guard in the speed limiter only, a hard minimum clearance), so they cannot be rerun
from the repository.

| | Variant | Recorded collision scenarios still colliding (of 7) | Arena: reach the goal (of 337) | Maps reached (of 13) | GA finds a collision |
|---|---|---|---|---|---|
| R | Original (24 rays, last scan) | 3 | 330 | 9 | **7 / 20** |
| R | Corner guard (g = 1.5) alone | 4 | 329 | 9 | 3 / 20 |
| R | Point memory 3 | 1 | 330 | 9 | 6 / 20 |
| R | Point memory 4 | 0 | 330 | 9 | 3 / 20 |
| R | Point memory 5 | 0 | 330 | 9 | 4 / 20 |
| R | **Corner guard (g = 1.5) + point memory 4 (the fix)** | **0** | 329 | 9 | **1 / 20** |
| R | Corner guard + point memory 6 | 0 | 327 | 9 | 2 / 20 |
| E | Corner guard, g = 2 | 0 of the 3 original | 329 | 8 (obstacle course deadlocks) | – |
| E | Corner guard, g = 1.0 or 1.25 | 1 of the 3 original | 330 | 9 | – |
| E | Corner guard only in the speed limiter (g = 2) | 1 of the 3 original | 275 (deadlocks) | 4 | – |
| E | Hard minimum clearance 0.08 m / 0.15 m (no memory) | 2 | 330 | 9 | 8 / 20, 6 / 20 |

The 7 scenarios are the 3 original collisions (the original controller collides in all of them) and 4 in
which the corner guard alone collided, found while regenerating RB1's tests (also in `fixed-issues/`, each
recording the configuration it collides with). The guard factor 1.5 is the smallest of 1.0, 1.25, 1.5, 1.75 and
2 that removes the three original collisions; 2 deadlocks on the obstacle course. Using the guard points only in
the speed limiter fails: heading selection then steers into gaps that the limiter refuses, and the robot
deadlocks. One-factor analyses of single scenarios (noise, dropout, 64 rays, before the fix):
`scripts/perception/ScenarioFactors.java <scenario.json>`.

## The fix

Default perception: corner guard (g = 1.5) and point memory 4, in both heading selection and the speed
limiter. Parameters `cornerGuard` (default `true`) and `pointMemory` (default 4);
`cornerGuard = false, pointMemory = 1` gives the earlier behaviour. Every faulty variant inherits it, so each
injected defect is still the only difference from the correct controller. The repair and improvement subjects
are generated from a template that now contains the same perception (`scripts/repair/generate_subjects.py`).
The corner guard adds one instrumented decision (`corner-pair`), so a coverage campaign now has 30 decision
outcomes instead of 28.

## Validation

| | Original perception | Fixed perception |
|---|---|---|
| GA finds a collision (20 runs, 2000 scenarios each) | 7 / 20 (35%) | **1 / 20 (5%)** |
| Recorded collision scenarios | 3 of 3 collide | 0 of 7 collide |
| Random arena scenarios reaching the goal | 330 / 337 | 329 / 337 |
| Maps reached (13) | 9 | 9 (same maps) |
| Experiment E reference (hand-written controller on the GP test maps) | 75%, mean cost 312 | 75%, mean cost 312.8 |

The difference in time to the first collision is significant (Mann–Whitney U, p = 0.029); the success rates
differ at Fisher's exact p = 0.044. `ReactivePerceptionTest` replays every fixed and known issue on each build.

**The fix does not make the controller collision-free.** The remaining collision (seed 19,
`examples/faulty-controllers/known-issues/reactive-residual-19.json`) penetrates by 5–7 mm, **even with perfect
sensing**. The original controller and a 64-ray robot both deadlock in that scenario instead. A provably safe
controller needs a different perception model, e.g. an occupancy grid that treats unobserved space as occupied.

## Consequences for test artefacts

- Regression records: BUG-1 and BUG-3 still reproduce. BUG-2 and BUG-4 no longer did and were regenerated
  from their experiments (`testing-bug2-ga`, seed 3; `testing-bug4-hc`, seed 5).
- Repair subject RB1: its failing tests were regenerated with `scripts/repair/Rb1FailureSearch.java`
  (12 tests; search suite 5 failing + 15 passing, held-out 5 + 30). The other subjects' suites: RB2 5 + 0 /
  5 + 0, RB3 5 + 4 / 5 + 8, RB4 and RB5 5 + 15 / 5 + 30. Templates now have 227–229 mutation points
  (156 before).
- During that search (34 seeds, 17 scenarios in which RB1 collides), the correct template (the reference fix
  of every subject) also collided in 2 scenarios in which the engine controller passed. The template differs from the engine controller on purpose
  (low-battery boost logic); such scenarios are dropped from the tests automatically.

## Reruns still to do

Run them one at a time with few threads (the machine has 16 GB; two suites in parallel ran out of memory):

```bash
java -Xmx2g -jar backend/java-engine/build/libs/robosbse-engine.jar suite examples/suites/reactive-controller.json --out experiments/reactive-rerun --threads 3
java -Xmx2g -jar backend/java-engine/build/libs/robosbse-engine.jar suite examples/suites/phase4-repair.json --out experiments/phase4 --threads 3
```
