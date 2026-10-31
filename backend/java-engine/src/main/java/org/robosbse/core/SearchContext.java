package org.robosbse.core;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.LongAdder;

/**
 * Everything a running algorithm interacts with: the problem, the random stream, and — most
 * importantly — the single {@link #evaluate} gateway.
 *
 * <p>Routing every fitness evaluation through the context gives uniform, trustworthy bookkeeping for
 * all algorithms: evaluation counting and budget enforcement, best-so-far tracking, the convergence
 * trace (best fitness after each improving evaluation), an archive of the best distinct
 * candidates, target hits (e.g. discovered failures), a Pareto archive and hypervolume trace for
 * multi-objective problems, behaviour coverage, pause/cancel handling and progress notification.
 * Algorithms only implement the search logic.
 *
 * <p><b>Parallel evaluation.</b> {@link #evaluateAll} evaluates a batch (e.g. a GA generation) on a
 * thread pool when the context was built with more than one thread and the problem is
 * {@link Problem#parallelSafe() parallel-safe}. Only the expensive part (repair and fitness) runs
 * concurrently; all bookkeeping happens afterwards on the calling thread, in batch order, with the same
 * stopping rules as one-by-one evaluation. A run therefore produces exactly the same result for every
 * thread count (time-limited runs excepted, because the time limit is checked once per batch).
 */
public final class SearchContext<S> {

    private static final int MAX_SERIES_POINTS = 4000;
    /** Behaviour coverage grid: cells per axis over the first two behaviour components. */
    public static final int COVERAGE_CELLS = 10;
    private static final ThreadMXBean THREADS = ManagementFactory.getThreadMXBean();

    private final Problem<S> problem;
    private final Budget budget;
    private final Rng rng;
    private final RunControl control;
    private final SearchObserver<S> observer;
    private final int archiveSize;
    private final int maxTargets;
    private final boolean stopOnTarget;
    private final long startNanos = System.nanoTime();

    private long evaluations;
    private long iteration;
    private Candidate<S> best;
    private Candidate<S> last;
    private final List<double[]> convergence = new ArrayList<>();
    private final Map<String, List<double[]>> series = new LinkedHashMap<>();
    private final Map<String, Double> stats = new LinkedHashMap<>();
    private final List<Candidate<S>> archive = new ArrayList<>();
    private final List<Candidate<S>> targets = new ArrayList<>();
    private long targetCount;
    private long firstTargetEvaluation = -1;
    private boolean targetReached;
    private final ParetoArchive<S> pareto;
    private final double[] referencePoint;
    private final long sampleInterval;
    private final BitSet behaviourCells = new BitSet(COVERAGE_CELLS * COVERAGE_CELLS);
    private boolean behaviourSeen;
    private final int threads;
    private ExecutorService pool;
    private final LongAdder workerCpuNanos = new LongAdder();

    private SearchContext(Builder<S> b) {
        this.problem = b.problem;
        this.budget = b.budget;
        this.rng = new Rng(b.seed);
        this.control = b.control == null ? new RunControl() : b.control;
        this.observer = b.observer == null ? SearchObserver.none() : b.observer;
        this.archiveSize = b.archiveSize;
        this.maxTargets = b.maxTargets;
        this.stopOnTarget = b.stopOnTarget;
        this.pareto = problem.objectiveNames().isEmpty() ? null : new ParetoArchive<>(b.paretoCapacity);
        this.referencePoint = problem.referencePoint();
        this.sampleInterval = Math.max(1, Math.min(budget.maxEvaluations(), 1_000_000) / 100);
        this.threads = Math.max(1, b.threads);
    }

    public static <S> Builder<S> builder(Problem<S> problem, Budget budget, long seed) {
        return new Builder<>(problem, budget, seed);
    }

    /**
     * Evaluates a solution, charging one unit of budget.
     *
     * @throws SearchTermination when the budget is exhausted, the time limit passed, the run was
     *                           cancelled, or a target was reached with stop-on-target enabled
     */
    public Candidate<S> evaluate(S solution) {
        control.checkpoint();
        if (targetReached && stopOnTarget) {
            throw new SearchTermination(StopReason.TARGET_REACHED);
        }
        if (evaluations >= budget.maxEvaluations()) {
            throw new SearchTermination(StopReason.BUDGET_EXHAUSTED);
        }
        if (budget.maxMillis() > 0 && elapsedMillis() >= budget.maxMillis()) {
            throw new SearchTermination(StopReason.TIME_LIMIT);
        }
        S repaired = problem.repair(solution);
        return record(repaired, problem.evaluate(repaired));
    }

    /**
     * Evaluates a batch of solutions, in parallel when this context has more than one thread, and
     * returns the candidates in input order. Equivalent to calling {@link #evaluate} on each solution
     * in turn: the same candidates are recorded, and the run stops at the same point when the budget
     * runs out or a target is reached with stop-on-target enabled (later results of the batch are then
     * discarded and not charged).
     */
    public List<Candidate<S>> evaluateAll(List<S> solutions) {
        List<Candidate<S>> out = new ArrayList<>(solutions.size());
        if (threads <= 1 || solutions.size() <= 1 || !problem.parallelSafe()) {
            for (S s : solutions) {
                out.add(evaluate(s));
            }
            return out;
        }
        control.checkpoint();
        if (targetReached && stopOnTarget) {
            throw new SearchTermination(StopReason.TARGET_REACHED);
        }
        if (budget.maxMillis() > 0 && elapsedMillis() >= budget.maxMillis()) {
            throw new SearchTermination(StopReason.TIME_LIMIT);
        }
        int n = (int) Math.min(solutions.size(), Math.max(0, remainingEvaluations()));
        if (n == 0) {
            throw new SearchTermination(StopReason.BUDGET_EXHAUSTED);
        }
        List<Future<Object[]>> futures = new ArrayList<>(n);
        ExecutorService executor = pool();
        for (int i = 0; i < n; i++) {
            S s = solutions.get(i);
            futures.add(executor.submit(() -> {
                long cpu = THREADS.getCurrentThreadCpuTime();
                S repaired = problem.repair(s);
                Evaluation evaluation = problem.evaluate(repaired);
                workerCpuNanos.add(Math.max(0, THREADS.getCurrentThreadCpuTime() - cpu));
                return new Object[] {repaired, evaluation};
            }));
        }
        List<Object[]> results = new ArrayList<>(n);
        for (Future<Object[]> f : futures) {
            results.add(join(f));
        }
        for (Object[] r : results) {
            if (targetReached && stopOnTarget) {
                throw new SearchTermination(StopReason.TARGET_REACHED);
            }
            @SuppressWarnings("unchecked")
            S repaired = (S) r[0];
            out.add(record(repaired, (Evaluation) r[1]));
        }
        if (n < solutions.size()) {
            throw new SearchTermination(StopReason.BUDGET_EXHAUSTED);
        }
        return out;
    }

    private static Object[] join(Future<Object[]> future) {
        try {
            return future.get();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new SearchTermination(StopReason.CANCELLED);
        } catch (ExecutionException e) {
            if (e.getCause() instanceof RuntimeException re) {
                throw re;
            }
            if (e.getCause() instanceof Error err) {
                throw err;
            }
            throw new IllegalStateException(e.getCause());
        }
    }

    private synchronized ExecutorService pool() {
        if (pool == null) {
            pool = Executors.newFixedThreadPool(threads, r -> {
                Thread t = new Thread(r, "robosbse-eval");
                t.setDaemon(true);
                return t;
            });
        }
        return pool;
    }

    /** Bookkeeping for one evaluated candidate; always runs on the calling thread. */
    private Candidate<S> record(S repaired, Evaluation evaluation) {
        evaluations++;
        Candidate<S> candidate = new Candidate<>(repaired, evaluation, evaluations);
        last = candidate;
        if (candidate.betterThan(best)) {
            best = candidate;
            convergence.add(new double[] {evaluations, candidate.fitness()});
            observer.onNewBest(this, candidate);
        }
        offerToArchive(candidate);
        if (pareto != null) {
            pareto.offer(candidate);
            if (referencePoint != null && evaluations % sampleInterval == 0) {
                series("hypervolume", evaluations, pareto.hypervolume(referencePoint));
            }
        }
        markBehaviour(evaluation);
        if (behaviourSeen && evaluations % sampleInterval == 0) {
            series("behaviourCoverage", evaluations, behaviourCoverage());
        }
        if (problem.isTarget(evaluation)) {
            targetCount++;
            if (firstTargetEvaluation < 0) {
                firstTargetEvaluation = evaluations;
            }
            if (targets.size() < maxTargets) {
                targets.add(candidate);
            }
            targetReached = true;
            observer.onTarget(this, candidate);
        }
        observer.onEvaluation(this, candidate);
        return candidate;
    }

    private void offerToArchive(Candidate<S> candidate) {
        if (archiveSize <= 0) {
            return;
        }
        if (archive.size() >= archiveSize
                && candidate.fitness() >= archive.get(archive.size() - 1).fitness()) {
            return;
        }
        for (Candidate<S> c : archive) {
            if (c.solution().equals(candidate.solution())) {
                return;
            }
        }
        int pos = 0;
        while (pos < archive.size() && archive.get(pos).fitness() <= candidate.fitness()) {
            pos++;
        }
        archive.add(pos, candidate);
        if (archive.size() > archiveSize) {
            archive.remove(archive.size() - 1);
        }
    }

    /** Marks the coverage-grid cell of the first two behaviour components (clamped to [0, 1]). */
    private void markBehaviour(Evaluation evaluation) {
        double[] b = evaluation.behaviour();
        if (b.length < 2) {
            return;
        }
        behaviourSeen = true;
        behaviourCells.set(cell(b[0]) * COVERAGE_CELLS + cell(b[1]));
    }

    private static int cell(double value) {
        if (!(value > 0)) {
            return 0;
        }
        return Math.min(COVERAGE_CELLS - 1, (int) (value * COVERAGE_CELLS));
    }

    /**
     * Fraction of the cells of a 10 x 10 grid over the first two behaviour components that some
     * evaluated candidate fell into, or NaN when the problem defines no behaviour descriptor.
     */
    public double behaviourCoverage() {
        return behaviourSeen ? behaviourCells.cardinality() / (double) (COVERAGE_CELLS * COVERAGE_CELLS) : Double.NaN;
    }

    /** Number of evaluation threads ({@code 1} means sequential evaluation). */
    public int threads() {
        return threads;
    }

    /** Records the latest value of an algorithm statistic (temperature, generation, ...). */
    public void stat(String key, double value) {
        stats.put(key, value);
    }

    /** Appends a point to a named series (e.g. population diversity per generation). */
    public void series(String key, double x, double y) {
        List<double[]> points = series.computeIfAbsent(key, k -> new ArrayList<>());
        points.add(new double[] {x, y});
        if (points.size() > MAX_SERIES_POINTS) {
            // Decimate by keeping every other point; the shape of the curve is preserved.
            List<double[]> kept = new ArrayList<>(points.size() / 2 + 1);
            for (int i = 0; i < points.size(); i += 2) {
                kept.add(points.get(i));
            }
            kept.add(points.get(points.size() - 1));
            points.clear();
            points.addAll(kept);
        }
    }

    public void nextIteration() {
        iteration++;
    }

    public long iteration() {
        return iteration;
    }

    public Problem<S> problem() {
        return problem;
    }

    public SolutionSpace<S> space() {
        return problem.space();
    }

    public Rng rng() {
        return rng;
    }

    public Budget budget() {
        return budget;
    }

    public long evaluations() {
        return evaluations;
    }

    public long remainingEvaluations() {
        return budget.maxEvaluations() - evaluations;
    }

    public Candidate<S> best() {
        return best;
    }

    public Candidate<S> last() {
        return last;
    }

    public Map<String, Double> stats() {
        return stats;
    }

    public long targetCount() {
        return targetCount;
    }

    public long firstTargetEvaluation() {
        return firstTargetEvaluation;
    }

    /** The run's Pareto-front approximation, or null for single-objective problems. */
    public ParetoArchive<S> pareto() {
        return pareto;
    }

    public long elapsedMillis() {
        return (System.nanoTime() - startNanos - control.pausedNanos()) / 1_000_000L;
    }

    SearchResult<S> result(StopReason reason, long callerCpuNanos) {
        synchronized (this) {
            if (pool != null) {
                pool.shutdownNow();
                pool = null;
            }
        }
        if (pareto != null && referencePoint != null) {
            series("hypervolume", evaluations, pareto.hypervolume(referencePoint));
        }
        if (behaviourSeen) {
            series("behaviourCoverage", evaluations, behaviourCoverage());
            stats.put("behaviourCoverage", behaviourCoverage());
        }
        long cpuMillis = (Math.max(0, callerCpuNanos) + workerCpuNanos.sum()) / 1_000_000L;
        return new SearchResult<>(best, evaluations, elapsedMillis(), reason, List.copyOf(convergence),
                copySeries(), Map.copyOf(stats), List.copyOf(archive), List.copyOf(targets), targetCount,
                firstTargetEvaluation, iteration, pareto == null ? List.of() : pareto.members(),
                pareto == null ? Double.NaN : pareto.hypervolume(referencePoint), cpuMillis, threads);
    }

    /** CPU time consumed by the current thread so far (ns), or -1 when the JVM cannot measure it. */
    static long currentThreadCpuNanos() {
        return THREADS.isCurrentThreadCpuTimeSupported() ? THREADS.getCurrentThreadCpuTime() : -1;
    }

    private Map<String, List<double[]>> copySeries() {
        Map<String, List<double[]>> copy = new LinkedHashMap<>();
        series.forEach((k, v) -> copy.put(k, List.copyOf(v)));
        return copy;
    }

    /** Fluent configuration of a context. */
    public static final class Builder<S> {
        private final Problem<S> problem;
        private final Budget budget;
        private final long seed;
        private RunControl control;
        private SearchObserver<S> observer;
        private int archiveSize = 10;
        private int maxTargets = 50;
        private boolean stopOnTarget;
        private int paretoCapacity = 100;
        private int threads = 1;

        private Builder(Problem<S> problem, Budget budget, long seed) {
            this.problem = problem;
            this.budget = budget;
            this.seed = seed;
        }

        public Builder<S> control(RunControl control) {
            this.control = control;
            return this;
        }

        public Builder<S> observer(SearchObserver<S> observer) {
            this.observer = observer;
            return this;
        }

        public Builder<S> archiveSize(int archiveSize) {
            this.archiveSize = archiveSize;
            return this;
        }

        public Builder<S> maxTargets(int maxTargets) {
            this.maxTargets = maxTargets;
            return this;
        }

        public Builder<S> stopOnTarget(boolean stopOnTarget) {
            this.stopOnTarget = stopOnTarget;
            return this;
        }

        public Builder<S> paretoCapacity(int paretoCapacity) {
            this.paretoCapacity = paretoCapacity;
            return this;
        }

        /** Evaluation threads for {@link SearchContext#evaluateAll}; 1 (the default) is sequential. */
        public Builder<S> threads(int threads) {
            this.threads = threads;
            return this;
        }

        public SearchContext<S> build() {
            return new SearchContext<>(this);
        }
    }
}
