# Phase 4 results

> **Update 2026-10-31.** The correct reactive controller (and the repair template derived from it) received a
> perception fix ([perception-fix.md](perception-fix.md)). The numbers below were measured **before** the fix.
> The Experiment E reference row is unchanged by it; Experiment F and genetic improvement still have to be
> rerun, and RB2–RB5 and genetic improvement have still never run as benchmarks.

Measured on 2026-10-28 (engine 0.1.0, Java 21.0.12, Windows 10, 8 logical processors, 16 GB RAM).
Configurations: `examples/benchmarks/experiment-E-gp.json`, `experiment-F-*.json`, `experiment-GI-reactive.json`;
all of them together: `examples/suites/phase4.json`.

```bash
java -jar backend/java-engine/build/libs/robosbse-engine.jar suite examples/suites/phase4.json --out experiments/phase4
```

**Status of the runs.** The Phase 4 suite completed Experiment E and Experiment F on subject RB1. It was then
stopped by the host because the machine ran low on memory: Experiment F ran 8 repair runs at a time, each with
its own candidate worker JVMs. Experiment F on RB2–RB5 and the genetic-improvement benchmark have **not been
run**. Their configurations now run 2 runs at a time. The observations for those subjects below come from
single exploratory runs during development (one seed, 300 candidates, partly with an earlier version of the
constant operator), not from the benchmark, and are labelled as such.

## Experiment E: genetic programming and generalisation

**Question.** Can genetic programming evolve simple reactive controllers that generalise beyond their training
environments?

**Setup.** GA (population 50) over typed controller programs (≤ 40 nodes, depth ≤ 7, parsimony 0.05 per node),
4 000 evaluations, 10 seeds per configuration. Training sets of 2 maps (empty field, obstacle course), 4 maps
(+ 2 generated) or 8 maps (+ 6 generated). Test set: 8 maps never seen by the search (warehouse, testing
arena, 6 generated maps). Baseline: random program search on the 8 training maps. Reference: the hand-written
reactive controller.

| Configuration | Runs solving every training map | Mean test-map success | Median test cost |
|---|---|---|---|
| GP, 2 training maps | 10/10 | 31% | 826 |
| GP, 4 training maps | 8/10 | 39% | 694 |
| GP, 8 training maps | 1/10 | 44% | 600 |
| Random programs, 8 training maps | 0/10 | 21% | 888 |
| Hand-written reactive controller | (3 of the 4 default training maps) | **75%** (6 of 8) | 312 |

Test cost (compared measure): GP with 8 maps vs random programs p(Holm) = 0.044, A12 = 0.86; no other pairwise
difference is significant (2 vs 8 maps: p(Holm) = 0.36). The best single program (8 training maps) reached 5 of
the 8 test maps:

```
if not goalAhead:
  if (goalDistance > goalBearing):
    if (front > 0.66):
      if (left > 0.56): veerLeft  else: forward
    else: turnRight
  else:
    if goalLeft: turnRight  else: forward
else: veerRight
```

**Interpretation.** GP fits its training maps easily, better than the hand-written controller does on the
default training set, and then **overfits**: with 2 training maps it reaches only 31% of unseen maps. More
training maps make the training task harder (only 1 of 10 runs solves all 8) and generalisation better (44%),
a consistent trend that 10 seeds do not make significant. No evolved program matches the hand-written
controller on the unseen maps (75%). The answer to the research question here is: partially. Evolved programs
generalise better than random programs, but not as well as a hand-designed controller, at this budget and
with this language.

## Experiment F: automated program repair

**Question.** Can search-based mutation discover valid patches for known controller defects? Random patch
search vs genetic search vs local search, 300 candidate evaluations, 10 seeds each.

### RB1: incorrect stopping-distance threshold (benchmark completed)

Search tests: 5 failing (found by search-based testing against the faulty subject) + 15 passing; held-out:
5 failing + 30 passing. 156 mutation points.

| Algorithm | Runs with a plausible patch | Median candidates to the first | Best patch passes held-out | Equivalent to the reference fix | Identical |
|---|---|---|---|---|---|
| Random patch search | 10/10 | 14 | 6/10 | 0 | 0 |
| Genetic search | 10/10 | 14 | 6/10 | 0 | 0 |
| Local search (stochastic HC) | 10/10 | 23 | 6/10 | 2 | 1 |

No significant difference in time to the first plausible patch (smallest p(Holm) = 0.18). Every best patch
minimised to one edit.

**Interpretation.** Plausible patches are cheap here and mostly wrong. Many single edits elsewhere (a sign in
the heading search, a heading constant, the corridor arithmetic) change the trajectories just enough to avoid
the five failing collisions, so the search tests are satisfied in about 14 candidates. Four in ten of those
best patches then fail held-out tests, and only local search found the actual fault (`200.0 → 2.0` once, an
equivalent edit once). This is the test-suite overfitting known from the APR literature (e.g. Smith et al.,
2015, "Is the cure worse than the disease?"), reproduced in a robot controller. RoboSBSE reports it rather than
counting plausible patches as repairs.

### RB2–RB5 (exploratory single runs only; benchmark not run)

- **RB2 (wrong comparison operator):** random and genetic search found the exact reference fix (`>=` → `<=`)
  and two equivalent alternatives (`<`, and negating the call site). Every scenario fails for this subject, so
  it has no passing tests.
- **RB3 (incorrect turn direction):** random search found an equivalent alternative: negating the steering
  gain's default (`2.5` → `−2.5`) cancels the inverted sign constant.
- **RB4 (missing boundary check):** plausible patches passed all 35 held-out tests but were not equivalent to
  the fix. They disabled the low-battery feature (`battery < 0.1` → `false`) or inverted the boost
  (`× 1.25` → `− 1.25`). Removing a feature is a valid-looking non-repair that held-out tests did not catch;
  the equivalence check did.
- **RB5 (incorrect sensor interpretation):** random search found no plausible patch within 300 candidates; the
  fix is one specific operator among about 770 single edits.

## Genetic improvement (benchmark not run)

Exploratory single runs (GA and NSGA-II, population 20, 300 candidates) improved the correct controller's mean
travel time by about 15–17% and its counted work per step by 6–19%. Every candidate passed all 20
correctness tests, yet the largest improvement (NSGA-II: time ratio 0.85, work ratio 0.81) **replaced the
braking-arc safety check by `v >= 0.03`**. The correctness suite did not notice; one of 40 held-out tests did.
Improvements that passed all 40 held-out tests reached a time ratio of about 0.84 with a work ratio of about
0.97. The result ranks improvements by held-out validation for exactly this reason.

## Finding: the correct reactive controller can collide

Generating RB1's failing tests also produced three scenarios in which the **correct** reactive controller
collides (penetration 0.6–1.4 cm), contradicting the Phase 1 observation that it never violated safety.
Varying one factor at a time:

| Scenario (seed) | As found | No sensor noise | No scan dropout | Neither | 64 rays |
|---|---|---|---|---|---|
| 8 | collision | collision | ok | ok | ok |
| 10 | collision | ok (deadlock) | ok (deadlock) | ok (deadlock) | ok |
| 30 | collision | collision | collision | collision | ok |

Cause: the controller's safety checks only know the obstacle points hit by its 24 range rays (15° apart), so
an obstacle corner between two rays is invisible to the braking-arc check. Scan dropout makes this more likely,
but seed 30 collides with perfect sensing. The scenarios are in `examples/faulty-controllers/known-issues/`.
The defect was first documented and not fixed, because every injected-bug variant and every earlier result
uses this controller family. It has since been mitigated, not removed:
[perception-fix.md](perception-fix.md).

## Threats to validity

- Experiment F covers one subject with a full benchmark; the other four have exploratory evidence only.
- Equivalence to the reference fix is judged on 35 held-out scenarios; identical behaviour there is not a
  proof of identical behaviour everywhere.
- Experiment E uses one language, one budget and generated maps from one generator; 10 seeds per configuration.
- Wall-clock times come from a busy desktop and are indicative only.
