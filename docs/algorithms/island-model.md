# Parallel evaluation and the island-model GA (`island-ga`)

## Parallel fitness evaluation

Simulation dominates the cost of every RoboSBSE run, and population-based algorithms evaluate whole
generations whose members are independent. `SearchContext.evaluateAll(batch)` therefore evaluates a batch on
a thread pool when the run has more than one evaluation thread (`options.threads`, the Lab's and Testing
page's "Evaluation threads", `run --eval-threads N`):

1. termination checks (cancel, time limit, target, budget) run once before the batch, and the batch is cut
   to the remaining budget;
2. only `repair` + `evaluate` run concurrently; each problem instance is immutable and every evaluation uses
   fresh simulator and controller objects (`Problem.parallelSafe()`, default true; a problem can opt out);
3. all bookkeeping (counting, best-so-far, convergence, archives, Pareto archive, targets, observers) then
   runs on the calling thread **in batch order**, with the same stopping rules as one-by-one evaluation.
   With stop-on-target, later results of the batch are discarded and not charged.

Consequence, tested for GA, ES, synchronous PSO, random search, NSGA-II, the memetic algorithm, novelty search
and the island GA: a run gives **bit-identical results for every thread count**. Only time-limited runs can
differ, because the time limit is checked once per batch.

Batching changes no random-number order because evaluation never draws from the search's random stream.
PSO is the exception that needed care: its default `update = immediate` lets a particle see personal bests
improved earlier in the same iteration, so it evaluates one particle at a time. `update = synchronous` moves
the whole swarm on the iteration-start bests and evaluates it as one batch.

Every run also records its **CPU time** (`cpuMs`: the running thread plus all evaluation threads, from the
JVM's per-thread CPU clocks) next to the wall-clock time (`elapsedMs`).

**Measured speed-up** (GA, population 40, reactive-controller tuning, 400 evaluations, 5 runs, 8 logical
processors): median wall time 14.2 s with 1 thread, 7.3 s with 2 (1.95×), 5.3 s with 4 (2.7×), 5.7 s with 8,
with identical results. CPU time grows with the thread count (13.8 s → 28.7 s at 8 threads), so beyond four
threads the extra threads only compete. See [Phase 3 results](../experiments/phase3-results.md).

Benchmarks already ran independent *runs* in parallel (`threads` of a benchmark). Use evaluation threads for
single runs, and benchmark threads for many runs; combining both oversubscribes the processor.

## Island-model GA

**Intuition.** Several sub-populations ("islands") evolve separately and occasionally exchange individuals.
Isolation preserves diversity: islands can settle on different regions, and migration spreads good
building blocks (Whitley et al.; Alba and Tomassini, 2002).

```
islands × islandSize individuals, one generational GA (with elitism) per island
every generation: breed on every island → evaluate all offspring as ONE batch → replace per island
every migrationInterval generations:
    emigrants of every island chosen first (best or random members)
    ring: island i → island i+1;   complete: every island → every other island
    immigrants replace the receiver's worst members (its best always survives; at most islandSize − 1)
```

- `migrationInterval = 0` gives isolated islands: independent GAs that share one budget.
- **Parallelism:** the offspring of all islands form one batch, so islands run in parallel with evaluation
  threads, deterministically.
- **Recorded:** the merged population's GA series, `islandBestSpread` (worst minus best island best),
  `meanIslandDiversity`, and the stats `islands`, `migrations`.
- **Result on the maze** (6 000 evaluations, 15 runs): panmictic GA of 100, four ring-connected islands of 25
  and four isolated islands showed no significant difference (Holm-corrected p ≥ 0.29), and none reached the
  goal. Whether islands help is problem-dependent; RoboSBSE measures it rather than assuming it.
