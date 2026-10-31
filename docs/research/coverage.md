# Coverage-guided testing of the reactive controller

Phase 3 adds white-box information to the search-based testing module: which decisions of the controller a
scenario exercised, and how close it came to the outcomes it missed.

## Instrumentation

The reactive controller (`ReactiveAvoidanceController`) and its four faulty variants carry **hand-placed
probes** (`testing.coverage.Probe`) on every explicit conditional of the class: `if`, `while` and `?:`. That is
14 decisions, plus one for BUG-3's faulty low-battery feature. Library calls such as `Math.min` and
fixed-count `for` loops are not counted as decisions. A probe returns the condition's value unchanged, so an
instrumented controller behaves exactly like an uninstrumented one (tested: identical steps, final pose,
energy and path). Without an attached trace a probe only computes the comparison.

| decision | condition | evaluated under |
|---|---|---|
| `scan-valid` | a range scan arrived this step | every step |
| `ray-hit` | `r < sensorRange` for a ray | `scan-valid:true` |
| `goal-reached` | completion check (Euclidean; BUG-4: Manhattan) | every step |
| `escape-improves` | `free > escapeFree` | `goal-reached:false` |
| `heading-free` | `free >= needed` | `goal-reached:false` |
| `has-previous-heading` | a heading was chosen before (hysteresis term) | `heading-free:true` |
| `better-heading` | `cost < chosenCost` | `heading-free:true` |
| `no-free-heading` | no candidate corridor is free (escape heading) | `goal-reached:false` |
| `turn-in-place` | `|heading error| > 1.2 rad` | `goal-reached:false` |
| `in-corridor` | `along > 0 && |lateral| < radius + margin` | `ray-hit:true` (loop entry) |
| `speed-loop` | `v >= 0.03` in the safety limiter | `goal-reached:false` |
| `arc-clear` | the braking arc at speed v is clear | `speed-loop:true` |
| `straight-arc` | `|omega / v| < 1e-6` | `speed-loop:true` |
| `arc-blocked` | an arc sample comes too close to a point | `speed-loop:true` |
| `low-battery-boost` (BUG-3 only) | `battery < 0.1` | `turn-in-place:false` |

Each decision has two outcomes, so the correct controller has 28 coverage targets and BUG-3 has 30. Coverage
is reported **only** for controllers that implement `Instrumented`. For the route-following controllers it is
"unavailable", and nothing is estimated.

This is source-level probe instrumentation of one class, not automatic bytecode instrumentation of the whole
code base. The decision list is explicit (and exported in the engine's capabilities), so the coverage number
always refers to it.

## Branch distance and approach level

For every decision a `CoverageTrace` keeps the number of evaluations and, per outcome, the **smallest branch
distance** observed during the episode (0 once taken). The distances follow Korel and Tracey et al.:

| predicate | distance to `true` (when false) | distance to `false` (when true) |
|---|---|---|
| `a < b` | `a − b + K` | `b − a` |
| `a <= b` | `a − b` | `b − a + K` |
| `A && B` | `d(A) + d(B)` over the false conjuncts | `min(d(¬A), d(¬B))` |
| boolean flag | 1 | 1 |

with `K = 1e-9`. The fitness of a target outcome combines both measures (Wegener et al.; McMinn 2004):

```
fitness = approachLevel + d / (d + 1)
```

If the target decision was evaluated, the approach level is 0 and `d` is its distance. Otherwise the
search walks up the declared parents: the approach level counts the decisions between the target and the
closest evaluated ancestor, and `d` is that ancestor's distance to the outcome that leads towards the target.

## Using coverage

- **Every test-generation run** reports the union of outcomes covered by all valid scenarios it generated
  (`coverage` in the result: per decision the number of scenarios that took each outcome). The Testing page
  shows it in the Coverage tab, and every replay lists the outcomes its scenario covered.
- **Branch-guided failure search** (`fitness = branch:<decision>:<true|false>`): first reach the outcome, then
  falsify a property:
  `1 + approachLevel + d/(d+1)` while uncovered (≥ 1), and `ρ / (1 + |ρ|)` once covered (in (−1, 1); negative =
  violation). Invalid scenarios get `100 + violation`.
- **Coverage campaigns** (`robosbse-engine coverage <campaign.json>`, the Coverage tab, protocol `coverage`):
  every outcome is a target, visited in declaration order. Targets already covered by earlier searches are
  skipped ("collateral"). Otherwise the selected generator runs with that target's branch fitness until it
  covers it or exhausts its share: the remaining budget divided equally among the targets still uncovered, so
  budget saved on easy targets goes to hard ones. A random-testing baseline with the same total budget is
  measured on the same instrumentation. Report: `robosbse.coverage/1` with per-target status, evaluations,
  first hit, approach level and distance of uncovered targets, and a replayable covering scenario.

## What it showed

- Most outcomes of this small controller are covered by any non-trivial episode: the first targeted search
  covered 28 of the 30 BUG-3 outcomes collaterally.
- `low-battery-boost:true` is rare when the initial battery is at least 0.2 (2 of 843 random valid
  scenarios). The targeted GA covered it after 127 evaluations.
- `no-free-heading:true` was covered neither by search nor by 6 000 random scenarios. Its best branch
  distance stayed at 2.5 m (= sensor range − largest free-distance setting): with four obstacle slots some
  direction always has a fully open corridor, so the distance has no gradient. The outcome is probably
  infeasible in this scenario space, but that is not proven.
- For finding BUG-3 itself, see Experiment C in [Phase 3 results](../experiments/phase3-results.md).

## Limitations

- Hand-placed probes must be kept in sync with the code. A missing probe is missing coverage, not a wrong
  number, but the decision list is only as complete as its author made it.
- Decision coverage of one class says nothing about paths, data flow or the simulator.
- Branch distances of boolean flags (`scan-valid`, `arc-clear`) carry no gradient.
- Covering a decision is not finding a defect: BUG-1 and BUG-2 live in hooks without new decisions.
