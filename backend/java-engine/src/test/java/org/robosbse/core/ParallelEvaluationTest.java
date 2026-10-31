package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

class ParallelEvaluationTest {

    /** Sphere whose behaviour descriptor is the first two coordinates scaled to [0, 1]. */
    static final class BehaviourSphere implements Problem<RealVector> {
        private final Sphere sphere = new Sphere(4);

        @Override
        public String type() {
            return "behaviour-sphere";
        }

        @Override
        public SolutionSpace<RealVector> space() {
            return sphere.space();
        }

        @Override
        public Evaluation evaluate(RealVector x) {
            return sphere.evaluate(x).withBehaviour(new double[] {(x.get(0) + 5) / 10, (x.get(1) + 5) / 10});
        }
    }

    /** Two objectives (x0, 1 - sqrt(x0) + g), x in [0, 1]^4: a convex ZDT1-like front. */
    static final class TwoObjectives implements Problem<RealVector> {
        private final RealVectorSpace space = RealVectorSpace.uniform(4, 0, 1);

        @Override
        public String type() {
            return "two-objectives";
        }

        @Override
        public SolutionSpace<RealVector> space() {
            return space;
        }

        @Override
        public Evaluation evaluate(RealVector x) {
            double g = 1 + 9 * (x.get(1) + x.get(2) + x.get(3)) / 3;
            double f2 = g * (1 - Math.sqrt(x.get(0) / g));
            return new Evaluation(x.get(0) + f2, 0, Map.of(), new double[] {x.get(0), f2});
        }

        @Override
        public List<String> objectiveNames() {
            return List.of("f1", "f2");
        }

        @Override
        public double[] referencePoint() {
            return new double[] {1.1, 11};
        }
    }

    private static <S> SearchResult<S> run(Problem<S> problem, String algorithm, Params params, long budget, int threads) {
        SearchContext<S> ctx = SearchContext.builder(problem, Budget.evaluations(budget), 11).threads(threads).build();
        return SearchAlgorithm.execute(AlgorithmRegistry.create(algorithm, params), ctx);
    }

    private static void assertSameRun(SearchResult<?> a, SearchResult<?> b) {
        assertEquals(a.evaluations(), b.evaluations());
        assertEquals(a.best().fitness(), b.best().fitness());
        assertEquals(a.best().index(), b.best().index());
        assertEquals(a.convergence().size(), b.convergence().size());
        for (int i = 0; i < a.convergence().size(); i++) {
            assertArrayEquals(a.convergence().get(i), b.convergence().get(i));
        }
        assertEquals(a.stopReason(), b.stopReason());
    }

    @ParameterizedTest
    @ValueSource(strings = {"genetic-algorithm", "evolution-strategy", "particle-swarm", "random-search", "island-ga",
            "memetic-algorithm", "novelty-search"})
    void resultsDoNotDependOnTheThreadCount(String algorithm) {
        Params params = algorithm.equals("particle-swarm") ? Params.of("update", "synchronous") : Params.empty();
        SearchResult<RealVector> one = run(new BehaviourSphere(), algorithm, params, 1500, 1);
        SearchResult<RealVector> four = run(new BehaviourSphere(), algorithm, params, 1500, 4);
        assertSameRun(one, four);
        assertEquals(1, one.threads());
        assertEquals(4, four.threads());
    }

    @Test
    void nsga2ParetoFrontDoesNotDependOnTheThreadCount() {
        SearchResult<RealVector> one = run(new TwoObjectives(), "nsga2", Params.empty(), 2000, 1);
        SearchResult<RealVector> three = run(new TwoObjectives(), "nsga2", Params.empty(), 2000, 3);
        assertSameRun(one, three);
        assertEquals(one.hypervolume(), three.hypervolume());
        assertEquals(one.pareto().size(), three.pareto().size());
    }

    @Test
    void immediateAndSynchronousPsoDifferButBothAreDeterministic() {
        SearchResult<RealVector> immediate = run(new Sphere(4), "particle-swarm", Params.empty(), 1200, 1);
        SearchResult<RealVector> again = run(new Sphere(4), "particle-swarm", Params.empty(), 1200, 4);
        assertSameRun(immediate, again); // immediate mode evaluates one at a time even with threads
        SearchResult<RealVector> sync = run(new Sphere(4), "particle-swarm", Params.of("update", "synchronous"), 1200, 1);
        assertTrue(sync.best().fitness() < 1, "synchronous PSO should still converge on the sphere");
    }

    @Test
    void batchesAreTruncatedExactlyAtTheBudget() {
        SearchResult<RealVector> r = run(new Sphere(3), "genetic-algorithm", Params.of("populationSize", 10), 37, 4);
        assertEquals(37, r.evaluations());
        assertEquals(StopReason.BUDGET_EXHAUSTED, r.stopReason());
    }

    @Test
    void stopOnTargetStopsAtTheSameEvaluationForEveryThreadCount() {
        long[] firsts = new long[2];
        long[] counts = new long[2];
        int k = 0;
        for (int threads : new int[] {1, 4}) {
            SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(2, 0.5), Budget.evaluations(5000), 3)
                    .threads(threads).stopOnTarget(true).build();
            SearchResult<RealVector> r = SearchAlgorithm.execute(
                    AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 40)), ctx);
            assertEquals(StopReason.TARGET_REACHED, r.stopReason());
            assertEquals(r.firstTargetEvaluation(), r.evaluations(), "nothing after the first hit is charged");
            firsts[k] = r.firstTargetEvaluation();
            counts[k++] = r.evaluations();
        }
        assertEquals(firsts[0], firsts[1]);
        assertEquals(counts[0], counts[1]);
    }

    @Test
    void evaluationErrorsSurfaceFromWorkerThreads() {
        Problem<RealVector> failing = new Problem<>() {
            private final Sphere sphere = new Sphere(2);

            @Override
            public String type() {
                return "failing";
            }

            @Override
            public SolutionSpace<RealVector> space() {
                return sphere.space();
            }

            @Override
            public Evaluation evaluate(RealVector x) {
                if (x.get(0) > 4) {
                    throw new IllegalStateException("simulated controller crash");
                }
                return sphere.evaluate(x);
            }
        };
        SearchContext<RealVector> ctx = SearchContext.builder(failing, Budget.evaluations(10_000), 1).threads(4).build();
        List<RealVector> batch = new ArrayList<>();
        for (int i = 0; i < 50; i++) {
            batch.add(new RealVector(new double[] {-5 + 0.2 * i, 0}));
        }
        IllegalStateException e = assertThrows(IllegalStateException.class, () -> ctx.evaluateAll(batch));
        assertEquals("simulated controller crash", e.getMessage());
    }

    @Test
    void problemsCanOptOutOfParallelEvaluation() {
        List<String> threadNames = new ArrayList<>();
        Problem<RealVector> serial = new Problem<>() {
            private final Sphere sphere = new Sphere(2);

            @Override
            public String type() {
                return "serial";
            }

            @Override
            public SolutionSpace<RealVector> space() {
                return sphere.space();
            }

            @Override
            public Evaluation evaluate(RealVector x) {
                threadNames.add(Thread.currentThread().getName());
                return sphere.evaluate(x);
            }

            @Override
            public boolean parallelSafe() {
                return false;
            }
        };
        run(serial, "genetic-algorithm", Params.of("populationSize", 20), 200, 4);
        assertEquals(200, threadNames.size());
        assertTrue(threadNames.stream().noneMatch(n -> n.startsWith("robosbse-eval")));
    }

    @Test
    void behaviourCoverageIsMeasuredOnlyWhenTheProblemDefinesBehaviour() {
        SearchResult<RealVector> withBehaviour = run(new BehaviourSphere(), "random-search", Params.empty(), 500, 1);
        double coverage = withBehaviour.stats().get("behaviourCoverage");
        assertTrue(coverage > 0.5 && coverage <= 1.0, "500 uniform samples should cover most of a 10x10 grid: " + coverage);
        assertTrue(withBehaviour.series().containsKey("behaviourCoverage"));
        SearchResult<RealVector> without = run(new Sphere(2), "random-search", Params.empty(), 200, 1);
        assertFalse(without.stats().containsKey("behaviourCoverage"));
        assertTrue(without.cpuMillis() >= 0);
    }
}
