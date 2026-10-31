# Faulty controllers

`ReactiveAvoidanceController` is the correct reference implementation. Each faulty variant overrides
**exactly one** protected hook, so every defect is isolated, documented and independently testable.
Source: `backend/java-engine/src/main/java/org/robosbse/simulation/controllers/faulty/`.

| Id | Defect class | The bug | Violates | Trigger | Difficulty for random testing (2000 scenarios) |
|---|---|---|---|---|---|
| `reactive-bug1-dropout` | incorrect sensor interpretation | a lost scan (`NaN` readings) is treated as free space: points dropped, full speed towards the goal | `collision` | high dropout near an obstacle | easy: ~6% of random scenarios fail; median first failure at evaluation 5 |
| `reactive-bug2-braking` | incorrect stopping-distance threshold | the safety arc checks `v·dt` instead of `v·dt + v²/(2a)` | `collision` | low free-distance setting, high speed, obstacle ahead | hard: masked by the conservative heading selection. **0 / 2000** random scenarios; 2/15 random runs vs 12/15 GA runs in the benchmark |
| `reactive-bug3-boost` | missing bound check | below 10% battery the speed limit is multiplied by 1.25 without clamping | `velocity` | battery < 0.1 and speed limit > 0.8 | plateau: ~3% of scenarios (56 / 2000), and robustness gives no gradient towards the trigger |
| `reactive-bug4-completion` | wrong distance metric (comparison) | completion uses `\|dx\|+\|dy\| ≤ 1.15·tol` (Manhattan) instead of the Euclidean check | `completion` | final approach within about 9° of an axis | easy: ~15% of scenarios |

The correct controller showed no safety violation in any of these campaigns (2 000 random scenarios plus
36 000 GA-generated scenarios across four fitness guides). The worst clearance it reached was 0.08 m.

## Regression records

`regressions/` holds one failure record per bug. Each was **discovered by a real search run** from
`examples/experiments/testing-bug*.json` and then minimised. `ControllerTestingTest` replays every file on
each build and requires that:

1. the faulty controller still violates the recorded property (`reproduced`), and
2. the correct controller does not (differential check).

```bash
java -jar backend/java-engine/build/libs/robosbse-engine.jar reproduce examples/faulty-controllers/regressions/bug2-braking-collision.json
# controller          : reactive-bug2-braking
# recorded violations : [collision]
# replayed violations : [collision] (outcome COLLISION)
# correct controller  : no violation (differential check passed)
# REPRODUCED
```

In the UI: *Search-Based Testing → Load failure file*. This replays the failure with sensor rays (lost scans
in red) and shows the reproduction verdict.

## Adding a defect

1. Subclass `ReactiveAvoidanceController` and override one hook (`onScanLost`, `goalReached`, `speedLimit`,
   `brakingLookahead`), or add a new hook to the base class.
2. Document the defect, the violated property and the trigger in the class Javadoc.
3. Register it in `ControllerRegistry` with `faultyOf`, `defect` and `expectedProperty`.
4. Discover it with a testing experiment, save the minimised record into `regressions/`, and add its id to
   `ControllerTestingTest.everyFaultyVariantHasARegressionRecord`.
