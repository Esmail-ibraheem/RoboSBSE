package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.baseline.RandomSearch;
import org.robosbse.core.representation.RealVector;

class SearchContextTest {

    @Test
    void budgetIsEnforcedExactly() {
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(3), Budget.evaluations(137), 1).build();
        SearchResult<RealVector> r = SearchAlgorithm.execute(new RandomSearch(), ctx);
        assertEquals(137, r.evaluations());
        assertEquals(StopReason.BUDGET_EXHAUSTED, r.stopReason());
        assertThrows(SearchTermination.class, () -> ctx.evaluate(ctx.space().random(ctx.rng())));
    }

    @Test
    void convergenceIsStrictlyImprovingAndEndsAtBest() {
        SearchResult<RealVector> r = SearchAlgorithm.execute(new RandomSearch(),
                SearchContext.builder(new Sphere(4), Budget.evaluations(500), 2).build());
        double previous = Double.POSITIVE_INFINITY;
        long previousEval = 0;
        for (double[] p : r.convergence()) {
            assertTrue(p[1] < previous, "best-so-far must strictly decrease");
            assertTrue(p[0] > previousEval, "evaluation indices must increase");
            previous = p[1];
            previousEval = (long) p[0];
        }
        assertEquals(r.best().fitness(), previous);
        assertEquals(r.best().fitness(), r.bestAt(500));
    }

    @Test
    void archiveIsSortedAndDistinct() {
        SearchResult<RealVector> r = SearchAlgorithm.execute(new RandomSearch(),
                SearchContext.builder(new Sphere(2), Budget.evaluations(300), 3).archiveSize(7).build());
        List<Candidate<RealVector>> archive = r.archive();
        assertEquals(7, archive.size());
        Set<RealVector> seen = new HashSet<>();
        for (int i = 0; i < archive.size(); i++) {
            assertTrue(seen.add(archive.get(i).solution()));
            if (i > 0) {
                assertTrue(archive.get(i - 1).fitness() <= archive.get(i).fitness());
            }
        }
        assertEquals(r.best().fitness(), archive.get(0).fitness());
    }

    @Test
    void stopOnTargetEndsTheRunAfterTheFirstHit() {
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(1, 4.0), Budget.evaluations(10_000), 4)
                .stopOnTarget(true).build();
        SearchResult<RealVector> r = SearchAlgorithm.execute(new RandomSearch(), ctx);
        assertEquals(StopReason.TARGET_REACHED, r.stopReason());
        assertEquals(r.firstTargetEvaluation(), r.evaluations());
        assertTrue(r.best().fitness() <= 4.0);
    }

    @Test
    void cancelStopsARunningSearch() throws Exception {
        RunControl control = new RunControl();
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(2), Budget.evaluations(Long.MAX_VALUE), 5)
                .control(control).build();
        AtomicReference<SearchResult<RealVector>> result = new AtomicReference<>();
        Thread t = new Thread(() -> result.set(SearchAlgorithm.execute(new RandomSearch(), ctx)));
        t.start();
        Thread.sleep(50);
        control.cancel();
        t.join(5000);
        assertFalse(t.isAlive());
        assertEquals(StopReason.CANCELLED, result.get().stopReason());
    }

    @Test
    void pauseBlocksEvaluationsUntilResume() throws Exception {
        RunControl control = new RunControl();
        control.pause();
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(2), Budget.evaluations(50), 6)
                .control(control).build();
        CountDownLatch done = new CountDownLatch(1);
        Thread t = new Thread(() -> {
            SearchAlgorithm.execute(new RandomSearch(), ctx);
            done.countDown();
        });
        t.start();
        assertFalse(done.await(150, TimeUnit.MILLISECONDS), "paused search must not finish");
        assertEquals(0, ctx.evaluations());
        control.resume();
        assertTrue(done.await(5, TimeUnit.SECONDS));
        assertEquals(50, ctx.evaluations());
        assertTrue(control.pausedNanos() >= 100_000_000L);
    }

    @Test
    void timeLimitStopsTheRun() {
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(2), new Budget(Long.MAX_VALUE, 30), 7).build();
        SearchResult<RealVector> r = SearchAlgorithm.execute(new RandomSearch(), ctx);
        assertEquals(StopReason.TIME_LIMIT, r.stopReason());
        assertTrue(r.elapsedMillis() >= 30);
    }

    @Test
    void sameSeedGivesIdenticalRuns() {
        SearchResult<RealVector> a = SearchAlgorithm.execute(new RandomSearch(),
                SearchContext.builder(new Sphere(3), Budget.evaluations(200), 99).build());
        SearchResult<RealVector> b = SearchAlgorithm.execute(new RandomSearch(),
                SearchContext.builder(new Sphere(3), Budget.evaluations(200), 99).build());
        assertEquals(a.best().solution(), b.best().solution());
        assertEquals(a.convergence().size(), b.convergence().size());
    }
}
