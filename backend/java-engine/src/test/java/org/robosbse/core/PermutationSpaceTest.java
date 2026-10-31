package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.robosbse.core.representation.Permutation;
import org.robosbse.core.representation.PermutationSpace;

class PermutationSpaceTest {

    private final PermutationSpace space = new PermutationSpace(7);

    @Test
    void permutationsRejectInvalidArrays() {
        assertThrows(IllegalArgumentException.class, () -> new Permutation(new int[] {0, 0, 1}));
        assertThrows(IllegalArgumentException.class, () -> new Permutation(new int[] {0, 3}));
    }

    @Test
    void everyMutationProducesADifferentValidPermutation() {
        Rng rng = new Rng(1);
        for (String name : PermutationSpace.MUTATIONS) {
            MutationOperator<Permutation> m = space.mutation(Params.of("mutation", name));
            for (int i = 0; i < 300; i++) {
                Permutation parent = space.random(rng);
                Permutation child = m.mutate(parent, rng);
                new Permutation(child.toArray()); // validates
                assertNotEquals(parent, child, name);
            }
        }
    }

    @Test
    void everyCrossoverProducesValidChildren() {
        Rng rng = new Rng(2);
        for (String name : PermutationSpace.CROSSOVERS) {
            CrossoverOperator<Permutation> x = space.crossover(Params.of("crossover", name));
            for (int i = 0; i < 300; i++) {
                Permutation a = space.random(rng);
                Permutation b = space.random(rng);
                for (Permutation c : x.cross(a, b, rng)) {
                    new Permutation(c.toArray());
                }
            }
        }
    }

    @Test
    void cycleCrossoverTakesEveryPositionFromAParent() {
        Rng rng = new Rng(3);
        CrossoverOperator<Permutation> cx = space.crossover(Params.of("crossover", "cycle"));
        for (int i = 0; i < 200; i++) {
            Permutation a = space.random(rng);
            Permutation b = space.random(rng);
            Permutation c = cx.cross(a, b, rng).get(0);
            for (int p = 0; p < 7; p++) {
                assertTrue(c.get(p) == a.get(p) || c.get(p) == b.get(p));
            }
        }
    }

    @Test
    void neighbourhoodSizesAndMoveAttributes() {
        Permutation p = Permutation.identity(7);
        assertEquals(21, space.neighbourhood(Params.of("neighbourhood", "swap")).all(p).size());
        assertEquals(36, space.neighbourhood(Params.of("neighbourhood", "insert")).all(p).size());
        assertEquals(21, space.neighbourhood(Params.of("neighbourhood", "inversion")).all(p).size());
        Set<Permutation> distinct = new HashSet<>(space.neighbourhood(Params.of("neighbourhood", "insert")).all(p));
        assertEquals(36, distinct.size(), "insert moves must not duplicate each other");
        for (Move<Permutation> m : space.neighbourhood(Params.of("neighbourhood", "swap")).moves(p)) {
            assertEquals(m.attribute(), m.inverse(), "a swap is undone by the same swap");
        }
    }

    @Test
    void enumerationListsAllPermutationsOnce() {
        PermutationSpace small = new PermutationSpace(5);
        Set<Permutation> seen = new HashSet<>();
        Iterator<Permutation> it = small.enumerate();
        while (it.hasNext()) {
            assertTrue(seen.add(it.next()));
        }
        assertEquals(120, seen.size());
    }

    @Test
    void operatorNamesAreCheckedPerRepresentation() {
        assertEquals("swap", SolutionSpace.operator(Params.empty(), "mutation", PermutationSpace.MUTATIONS, "permutation"));
        assertThrows(IllegalArgumentException.class, () -> space.mutation(Params.of("mutation", "gaussian")));
        assertThrows(IllegalArgumentException.class, () -> space.crossover(Params.of("crossover", "one-point")));
    }

    @Test
    void distanceIsTheNormalisedHammingDistance() {
        Permutation a = new Permutation(new int[] {0, 1, 2, 3});
        Permutation b = new Permutation(new int[] {1, 0, 2, 3});
        assertEquals(0.5, new PermutationSpace(4).distance(a, b));
        assertEquals(0.0, new PermutationSpace(4).distance(a, a));
        assertEquals(List.of(1, 0, 2, 3).get(0), b.get(0));
    }
}
