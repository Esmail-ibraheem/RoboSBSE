# Program repair and genetic improvement

Both modules search over **source patches** to a Java controller. They share one machinery: a catalogue
of mutation points, a restricted set of patch operators, isolated compilation and testing, and patch
minimisation. Repair starts from a faulty controller and looks for a patch that passes its tests; genetic
improvement starts from a working controller and looks for one that is faster or cheaper while it keeps
passing its tests.

## Subjects

Six Java controllers live in `examples/faulty-controllers/repair/src/subjects/`. They are generated from one
template (map-free reactive navigation: heading search over free corridors, a braking-distance speed limiter
and a low-battery boost clamped to the robot's maximum speed), so every faulty subject differs from the
correct `ReactiveBaseline` in exactly its documented defect. One defect for each of the specification's examples:

| Subject | Defect kind | Defect | Reference fix |
|---|---|---|---|
| RB1 `StoppingDistanceBug` | Incorrect stopping-distance threshold | braking term `v² / (200.0 × decel)` instead of `2.0` | `200.0` → `2.0` |
| RB2 `ComparisonBug` | Wrong comparison operator | completion check `>=` instead of `<=` | `>=` → `<=` |
| RB3 `TurnDirectionBug` | Incorrect turn direction | steering sign constant `-1.0` | `-1.0` → `1.0` |
| RB4 `BoundaryCheckBug` | Missing boundary check | speed clamp only above `maxSpeed × 1.5` | drop `× 1.5` |
| RB5 `SensorMirrorBug` | Incorrect sensor interpretation | ray angle `θ − α` instead of `θ + α` (obstacles mirrored) | `−` → `+` |

The reference fix is stored with each subject (`robosbse.repair-subject/1`). It is used only to judge results,
never by the search. For RB1–RB3 and RB5 it is one allowed edit away (tested). For RB4 the closest allowed
edit is `1.5 → 1.0`, which is equivalent but not textually identical.

## Mutation points and operators

`SourceCatalog` parses the subject with JavaParser and lists, in source order and within the configured scope
(the whole class for repair; the heading search and speed limiter for improvement):

| Kind | Alternatives |
|---|---|
| comparison operator | the other five of `< <= > >= == !=` |
| arithmetic operator | the other three of `+ - * /` (not on string concatenation) |
| logical operator | the other of `&& ||` |
| numeric constant | double: 0, 1, ×0.01, ×0.1, ×0.5, ×2, ×10, ×100, −x; int: 0, 1, x±1, x/2, 2x, −x |
| condition of `if`, loops, `?:` | `true` (not for loops), `false`, its negation, another condition of the same method |

A patch is a set of edits, at most one per point; an edit inside an edited condition is a conflict and never
generated. Patches are applied to a fresh parse and printed with JavaParser's lexical-preserving printer, so a
patched file differs from the original only where it was edited (tested). The operators cannot add calls,
statements or identifiers. The subjects have 154–156 points (about 755 single edits each); the improvement scope
has 123 points.

## Isolated execution

Candidates never run inside the engine. `WorkerPool` starts child JVMs (`CandidateWorker`) with
`-Xmx256m`, one CPU hint and the serial collector. A worker compiles each candidate with `javax.tools` into a
fresh directory, loads it with a fresh class loader, runs the tests through the normal simulator and oracle,
and discards anything the candidate prints. The parent waits at most `timeoutMs` (10 s); on timeout the worker
is killed and replaced and the candidate counts as failing every test. A crash (exit, out of memory) is
handled the same way. Results are cached by source text, so re-evaluating a patch costs nothing. Java 21 has
no security manager, so workers are not file-system sandboxes; the restricted operators are what bounds
candidate behaviour, and the process boundary contains loops, crashes and memory exhaustion. Typical cost:
0.1–0.2 s per new candidate.

**Memory.** Each worker JVM needs up to about 0.3–0.4 GB, and a repair run holds a search worker per
`workers` plus a held-out worker. Parallel benchmark runs multiply that. The bundled repair and improvement
benchmarks therefore run 2 runs at a time; with 8 parallel runs the Phase 4 suite exhausted the memory of a
16 GB desktop that was also running other applications, and was stopped.

## Tests

A test is a scenario of the testing arena plus an expectation: no selected safety property may be violated,
and if the trusted reference controller reaches the goal there, the candidate must reach it too. The second
condition stops "patches" such as never moving, which pass every safety test.

- Scenarios come from the subject's failure records and from seeded random samples (200 per pool). A
  scenario is used only if the reference controller violates nothing in it and the subject's reference fix
  passes it.
- **Failing tests** are those the faulty subject fails (5 used by the search); **passing tests** those it
  passes (15, the regression suite). A disjoint pool gives **held-out tests** (5 failing + 30 passing) that the
  search never sees.
- RB1's defect is invisible to random scenarios (braking rarely matters). Its failing tests were found by
  search-based testing against the faulty subject (GA on collision robustness, 2 000 evaluations per seed),
  exactly as Phase 1 found BUG-2: six seed search tests, six held out, plus Phase 1's BUG-2 scenario.

## Repair fitness and outputs

`fitness = 2 × Σ loss(failing tests) + Σ loss(passing tests) + 0.01 × edits`. A test's loss is 0 when it
passes, else 1 + severity (worst violation depth, or the fraction of the distance left to the goal), and 2
when the candidate did not compile, timed out or crashed. A **plausible** patch passes every search test.

Every plausible patch in the result is:
1. **minimised**: ddmin over its edits, keeping plausibility;
2. **validated** on the held-out tests;
3. compared with the reference fix: **equivalent** when every held-out test gives the same verdict, outcome
   and episode time; **identical** when the patched source equals the fixed source up to whitespace;
4. **ranked**: passes held-out tests, then equivalent, then fewer edits.

The UI shows the ranked list and the original and patched code side by side.

**Passing a test suite does not prove a patch correct.** The categories above exist because they differ in
practice: Phase 4 found plausible patches that pass every held-out test but delete the faulty feature
instead of fixing it (RB4), or make the robot more cautious than the reference fix (RB1).

## Genetic improvement

`ImprovementProblem` searches the same patch space over `ReactiveBaseline`. Edits are restricted to the
heading search, the speed limiter and two constants. A candidate is **correct** if it passes all 20
correctness tests (scenarios the baseline passes, goal required wherever the baseline reaches it) and still
completes every performance map the baseline completes (obstacle course, warehouse, empty field). For
correct candidates:

- `timeRatio` = mean travel time on the performance maps / baseline;
- `workRatio` = counted work units per control step / baseline. Work is counted by the subject itself
  (`WorkCounter`: obstacle-point checks in the heading search and braking-arc check), a deterministic
  stand-in for computational cost. Wall-clock nanoseconds per step are reported too, but are noisy and not
  used.

`fitness = timeRatio + workRatio + 0.001 × edits` (baseline = 2); incorrect candidates score 10 + failures.
NSGA-II optimises (timeRatio, workRatio). Results report the baseline and the best candidate side by side,
the number of candidate evaluations and the test results. Every improvement is validated on 40 held-out
tests and ranked by that validation, because a correctness suite can miss a broken safety check (see the results).

## Limitations

- The operator set is deliberately small; defects that need new statements are out of reach.
- Tests are simulation scenarios; equivalence is judged on the held-out scenarios only.
- No fault localisation: every point is equally likely to be mutated.
- Workers are process-isolated with time and heap limits, not OS sandboxes.
