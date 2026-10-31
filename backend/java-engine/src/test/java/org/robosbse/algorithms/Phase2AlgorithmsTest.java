package org.robosbse.algorithms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.local.TabuSearch;
import org.robosbse.core.Budget;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.StopReason;
import org.robosbse.core.representation.Permutation;
import org.robosbse.core.representation.PermutationSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

class Phase2AlgorithmsTest {

    /** Travelling-salesman tour over points on a circle: the optimum visits them in angular order. */
    static final class CircleTour implements Problem<Permutation> {
        private final int n;
        private final PermutationSpace space;

        CircleTour(int n) {
            this.n = n;
            this.space = new PermutationSpace(n);
        }

        double optimum() {
            return n * 2 * Math.sin(Math.PI / n);
        }

        @Override
        public String type() {
            return "circle-tour";
        }

        @Override
        public SolutionSpace<Permutation> space() {
            return space;
        }

        @Override
        public Evaluation evaluate(Permutation p) {
            double length = 0;
            for (int i = 0; i < n; i++) {
                double a = 2 * Math.PI * p.get(i) / n;
                double b = 2 * Math.PI * p.get((i + 1) % n) / n;
                length += Math.hypot(Math.cos(a) - Math.cos(b), Math.sin(a) - Math.sin(b));
            }
            return Evaluation.of(length);
        }
    }

    /** ZDT1 (Zitzler, Deb, Thiele 2000): convex Pareto front f2 = 1 - sqrt(f1) at g = 1. */
    static final class Zdt1 implements Problem<RealVector> {
        private final RealVectorSpace space = RealVectorSpace.uniform(6, 0, 1);

        @Override
        public String type() {
            return "zdt1";
        }

        @Override
        public SolutionSpace<RealVector> space() {
            return space;
        }

        @Override
        public Evaluation evaluate(RealVector x) {
            double g = 0;
            for (int i = 1; i < x.size(); i++) {
                g += x.get(i);
            }
            g = 1 + 9 * g / (x.size() - 1);
            double f1 = x.get(0);
            double f2 = g * (1 - Math.sqrt(f1 / g));
            return new Evaluation(f1 + f2, 0, Map.of(), new double[] {f1, f2});
        }

        @Override
        public List<String> objectiveNames() {
            return List.of("f1", "f2");
        }

        @Override
        public double[] referencePoint() {
            return new double[] {1, 1};
        }
    }

    private static <S> SearchResult<S> run(String id, Params params, Problem<S> problem, long budget, long seed) {
        return SearchAlgorithm.execute(AlgorithmRegistry.create(id, params),
                SearchContext.builder(problem, Budget.evaluations(budget), seed).build());
    }

    // --- Tabu search -------------------------------------------------------------------------

    @Test
    void tabuMovesAreRejectedUnlessAspirationApplies() {
        TabuSearch.TabuList tabu = new TabuSearch.TabuList();
        tabu.add("m1", 10);
        List<Object> attrs = List.of("m1", "m2");
        // m1 is better but tabu and not better than the best so far -> m2 is chosen
        TabuSearch.Choice c = TabuSearch.select(List.of(1.0, 2.0), attrs, tabu, 5, 0.5, true);
        assertEquals(1, c.index());
        assertFalse(c.aspiration());
        // m1 would be a new best -> aspiration overrides its tabu status
        c = TabuSearch.select(List.of(0.4, 2.0), attrs, tabu, 5, 0.5, true);
        assertEquals(0, c.index());
        assertTrue(c.aspiration());
        // without aspiration the tabu move stays forbidden
        assertEquals(1, TabuSearch.select(List.of(0.4, 2.0), attrs, tabu, 5, 0.5, false).index());
        // after the tenure the move is free again
        assertEquals(0, TabuSearch.select(List.of(1.0, 2.0), attrs, tabu, 11, 0.5, false).index());
    }

    @Test
    void whenEverythingIsTabuTheSoonestExpiringMoveIsForced() {
        TabuSearch.TabuList tabu = new TabuSearch.TabuList();
        tabu.add("a", 20);
        tabu.add("b", 12);
        TabuSearch.Choice c = TabuSearch.select(List.of(1.0, 3.0), List.of("a", "b"), tabu, 10, 0.0, true);
        assertTrue(c.forced());
        assertEquals(1, c.index());
    }

    @Test
    void tabuSearchAcceptsWorseMovesAndSolvesATour() {
        CircleTour tour = new CircleTour(9);
        SearchResult<Permutation> r = run("tabu-search", Params.of("neighbourhood", "inversion", "tenure", 5), tour, 6000, 1);
        assertEquals(tour.optimum(), r.best().fitness(), 1e-9);
        List<double[]> path = r.series().get("current");
        boolean worsened = false;
        for (int i = 1; i < path.size(); i++) {
            worsened |= path.get(i)[1] > path.get(i - 1)[1];
        }
        assertTrue(worsened, "tabu search must leave local optima by accepting worse moves");
    }

    @Test
    void tabuSearchWorksOnRealVectors() {
        SearchResult<RealVector> r = run("tabu-search", Params.of("stepSize", 0.002), new Sphere(3), 20_000, 2);
        assertTrue(r.best().fitness() < 0.01, "reached " + r.best().fitness());
    }

    // --- Exhaustive search -----------------------------------------------------------------------

    @Test
    void exhaustiveSearchProvesTheOptimumOnASmallInstance() {
        CircleTour tour = new CircleTour(7);
        SearchResult<Permutation> r = run("exhaustive-search", Params.empty(), tour, 10_000, 1);
        assertEquals(5040, r.evaluations());
        assertEquals(StopReason.ALGORITHM_FINISHED, r.stopReason());
        assertEquals(1.0, r.stats().get("complete"));
        assertEquals(tour.optimum(), r.best().fitness(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> run("exhaustive-search", Params.empty(), new Sphere(2), 10, 1));
    }

    // --- PSO and ES --------------------------------------------------------------------------------

    @Test
    void particleSwarmSolvesTheSphereWithBothTopologies() {
        for (String topology : List.of("global", "ring")) {
            SearchResult<RealVector> r = run("particle-swarm", Params.of("topology", topology), new Sphere(5), 15_000, 3);
            assertTrue(r.best().fitness() < 1e-3, topology + " reached " + r.best().fitness());
        }
    }

    @Test
    void particleSwarmRefusesPermutations() {
        assertThrows(IllegalArgumentException.class, () -> run("particle-swarm", Params.empty(), new CircleTour(6), 100, 1));
        assertFalse(AlgorithmRegistry.get("particle-swarm").supports("permutation", false));
    }

    @Test
    void evolutionStrategiesSolveTheSphere() {
        for (String control : List.of("self-adaptive", "one-fifth")) {
            for (String selection : List.of("comma", "plus")) {
                SearchResult<RealVector> r = run("evolution-strategy",
                        Params.of("stepControl", control, "selection", selection), new Sphere(5), 15_000, 4);
                assertTrue(r.best().fitness() < 1e-3, control + "/" + selection + " reached " + r.best().fitness());
                assertTrue(r.stats().get("meanSigma") >= 1e-5);
            }
        }
    }

    @Test
    void plusSelectionIsElitist() {
        SearchResult<RealVector> r = run("evolution-strategy", Params.of("selection", "plus"), new Sphere(4), 4000, 5);
        List<double[]> best = r.series().get("populationBest");
        for (int i = 1; i < best.size(); i++) {
            assertTrue(best.get(i)[1] <= best.get(i - 1)[1]);
        }
    }

    // --- Steady-state GA --------------------------------------------------------------------------

    @Test
    void steadyStateGaNeverLosesItsBestAndWorksOnPermutations() {
        SearchResult<RealVector> r = run("steady-state-ga", Params.of("populationSize", 30), new Sphere(4), 8000, 6);
        List<double[]> best = r.series().get("populationBest");
        for (int i = 1; i < best.size(); i++) {
            assertTrue(best.get(i)[1] <= best.get(i - 1)[1]);
        }
        assertTrue(r.best().fitness() < 0.05);
        CircleTour tour = new CircleTour(8);
        SearchResult<Permutation> t = run("steady-state-ga", Params.of("crossover", "order", "mutation", "inversion"),
                tour, 6000, 6);
        assertEquals(tour.optimum(), t.best().fitness(), 1e-9);
    }

    // --- NSGA-II ---------------------------------------------------------------------------------

    @Test
    void nsga2ApproximatesTheZdt1Front() {
        SearchResult<RealVector> r = run("nsga2", Params.of("populationSize", 40), new Zdt1(), 8000, 7);
        // The true front has hypervolume 2/3 for reference point (1, 1).
        assertTrue(r.hypervolume() > 0.60, "hypervolume " + r.hypervolume());
        assertTrue(r.pareto().size() >= 20);
        for (var a : r.pareto()) {
            for (var b : r.pareto()) {
                assertFalse(org.robosbse.core.Pareto.dominates(a.evaluation().objectives(), b.evaluation().objectives()));
            }
        }
        List<double[]> hv = r.series().get("hypervolume");
        assertTrue(hv.get(hv.size() - 1)[1] >= hv.get(0)[1], "archive hypervolume can only grow");
    }

    @Test
    void nsga2BeatsRandomSearchOnHypervolume() {
        double nsga = run("nsga2", Params.empty(), new Zdt1(), 4000, 8).hypervolume();
        double random = run("random-search", Params.empty(), new Zdt1(), 4000, 8).hypervolume();
        assertTrue(nsga > random, nsga + " vs " + random);
    }

    @Test
    void nsga2NeedsObjectives() {
        assertThrows(IllegalArgumentException.class, () -> run("nsga2", Params.empty(), new Sphere(2), 100, 1));
        assertFalse(AlgorithmRegistry.get("nsga2").supports("real-vector", false));
    }
}
