package org.robosbse.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;

class ParamsTest {

    @Test
    void typedGettersConvertAndFallBack() {
        Params p = Params.of("a", 3, "b", "2.5", "c", true, "d", "x");
        assertEquals(3, p.getInt("a", 0));
        assertEquals(2.5, p.getDouble("b", 0));
        assertTrue(p.getBool("c", false));
        assertEquals("x", p.getString("d", ""));
        assertEquals(7, p.getInt("missing", 7));
        assertThrows(IllegalArgumentException.class, () -> Params.of("a", 2.5).getInt("a", 0));
    }

    @Test
    void validationRejectsUnknownAndOutOfRange() {
        List<ParamSpec> specs = List.of(ParamSpec.integer("n", 5, 1, 10, ""),
                ParamSpec.choice("mode", List.of("x", "y"), ""));
        assertTrue(ParamSpec.validate(specs, Params.of("n", 3, "mode", "y")).isEmpty());
        assertEquals(1, ParamSpec.validate(specs, Params.of("n", 30)).size());
        assertEquals(1, ParamSpec.validate(specs, Params.of("mode", "z")).size());
        assertEquals(1, ParamSpec.validate(specs, Params.of("typo", 1)).size());
    }

    @Test
    void registryFailsFastOnBadParameters() {
        assertThrows(IllegalArgumentException.class,
                () -> AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 0)));
        assertThrows(IllegalArgumentException.class,
                () -> AlgorithmRegistry.create("hill-climbing", Params.of("varient", "first")));
        assertThrows(IllegalArgumentException.class, () -> AlgorithmRegistry.create("nope", Params.empty()));
    }
}
