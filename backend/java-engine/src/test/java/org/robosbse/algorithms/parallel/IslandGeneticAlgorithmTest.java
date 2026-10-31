package org.robosbse.algorithms.parallel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.Sphere;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Candidate;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;

class IslandGeneticAlgorithmTest {

    /** Islands of 4 members with fitness island*10 + member (lower = better). */
    private static List<List<Candidate<RealVector>>> islands(SearchContext<RealVector> ctx, int count) {
        List<List<Candidate<RealVector>>> out = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            List<Candidate<RealVector>> island = new ArrayList<>();
            for (int m = 0; m < 4; m++) {
                // Sphere(1) centre is 1: x = 1 + sqrt(f) gives fitness f.
                island.add(ctx.evaluate(new RealVector(new double[] {1 + Math.sqrt(i * 10 + m)})));
            }
            out.add(island);
        }
        return out;
    }

    private static double best(List<Candidate<RealVector>> island) {
        return island.stream().mapToDouble(Candidate::fitness).min().orElseThrow();
    }

    @Test
    void ringMigrationSendsEachIslandsBestToItsSuccessorReplacingTheWorst() {
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(1), Budget.evaluations(100), 1).build();
        List<List<Candidate<RealVector>>> pops = islands(ctx, 3);
        IslandGeneticAlgorithm ga = new IslandGeneticAlgorithm(Params.of("islands", 3, "islandSize", 4, "migrants", 1));
        ga.migrate(ctx, pops);
        // Island 1 receives island 0's best (fitness 0) in place of its worst (13).
        assertEquals(0, best(pops.get(1)), 1e-9);
        assertTrue(pops.get(1).stream().noneMatch(c -> Math.abs(c.fitness() - 13) < 1e-9));
        // Island 0 receives island 2's best (20); its own best survives.
        assertEquals(0, best(pops.get(0)), 1e-9);
        assertTrue(pops.get(0).stream().anyMatch(c -> Math.abs(c.fitness() - 20) < 1e-9));
        pops.forEach(p -> assertEquals(4, p.size()));
    }

    @Test
    void completeTopologyNeverReplacesTheReceiversBest() {
        SearchContext<RealVector> ctx = SearchContext.builder(new Sphere(1), Budget.evaluations(100), 1).build();
        List<List<Candidate<RealVector>>> pops = islands(ctx, 4);
        IslandGeneticAlgorithm ga = new IslandGeneticAlgorithm(Params.of("islands", 4, "islandSize", 4, "migrants", 2,
                "topology", "complete"));
        ga.migrate(ctx, pops);
        for (List<Candidate<RealVector>> p : pops) {
            assertEquals(4, p.size());
            assertEquals(0, best(p), 1e-9, "island 0's best reaches every island");
        }
    }

    @Test
    void isolatedIslandsNeverMigrate() {
        SearchResult<RealVector> r = SearchAlgorithm.execute(
                AlgorithmRegistry.create("island-ga", Params.of("migrationInterval", 0)),
                SearchContext.builder(new Sphere(4), Budget.evaluations(2000), 3).build());
        assertEquals(0, r.stats().get("migrations"));
        SearchResult<RealVector> migrating = SearchAlgorithm.execute(
                AlgorithmRegistry.create("island-ga", Params.of("migrationInterval", 5)),
                SearchContext.builder(new Sphere(4), Budget.evaluations(2000), 3).build());
        assertTrue(migrating.stats().get("migrations") > 0);
        assertTrue(migrating.best().fitness() < 1);
    }

    @Test
    void invalidConfigurationsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new IslandGeneticAlgorithm(Params.of("migrants", 25)));
        assertThrows(IllegalArgumentException.class, () -> new IslandGeneticAlgorithm(Params.of("topology", "star")));
    }
}
