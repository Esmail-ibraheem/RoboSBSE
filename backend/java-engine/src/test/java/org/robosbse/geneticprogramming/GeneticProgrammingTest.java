package org.robosbse.geneticprogramming;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;

class GeneticProgrammingTest {

    static final String STEER = "(if goalAhead forward (if goalLeft turnLeft turnRight))";

    @Test
    void programsPrintAndParseBack() {
        Program p = Program.parse("(if (< front 0.8) turnLeft (if (and goalAhead (not obstacleAhead)) forward veerRight))", 40, 7);
        assertEquals(p, Program.parse(p.toString(), 40, 7));
        assertEquals(12, p.size());
        assertEquals(4, p.depth());
        assertTrue(p.pretty().contains("if (front < 0.80):"));
    }

    @Test
    void invalidProgramsAreRejectedNotRun() {
        assertThrows(IllegalArgumentException.class, () -> Program.parse("(if front forward stop)", 40, 7), "NUM where BOOL is needed");
        assertThrows(IllegalArgumentException.class, () -> Program.parse("(< front 1.0)", 40, 7), "root must be an action");
        assertThrows(IllegalArgumentException.class, () -> Program.parse("(jump forward)", 40, 7), "unknown function");
        assertThrows(IllegalArgumentException.class, () -> Program.parse("(if goalLeft forward)", 40, 7), "arity");
        assertThrows(IllegalArgumentException.class, () -> Program.parse(STEER, 4, 7), "size limit");
        assertThrows(IllegalArgumentException.class, () -> Program.parse(STEER, 40, 1), "depth limit");
        assertThrows(IllegalArgumentException.class, () -> Program.parse("(if goalLeft forward stop) extra", 40, 7));
    }

    @Test
    void operatorsKeepProgramsWellTypedAndWithinLimits() {
        ProgramSpace space = new ProgramSpace(30, 6, 2, 4);
        Rng rng = new Rng(3);
        var mutation = space.mutation(Params.empty());
        var crossover = space.crossover(Params.empty());
        Program a = space.random(rng);
        Program b = space.random(rng);
        for (int i = 0; i < 500; i++) {
            Program m = mutation.mutate(a, rng);
            assertTrue(space.withinLimits(m.root()));
            assertNotEquals(a, m, "mutation never returns a copy");
            for (Program c : crossover.cross(a, b, rng)) {
                assertTrue(space.withinLimits(c.root()));
                assertEquals(Op.Type.ACTION, c.root().type());
            }
            a = m;
            b = space.random(rng);
        }
        for (Program n : space.neighbourhood(Params.empty()).all(Program.parse(STEER, 30, 6))) {
            assertTrue(space.withinLimits(n.root()));
        }
        assertEquals(0.0, space.distance(a, a));
        assertTrue(space.distance(a, b) > 0 && space.distance(a, b) <= 1);
    }

    private static ControllerSynthesisProblem problem(String maps) {
        return new ControllerSynthesisProblem(List.of(MapLibrary.builtin(maps)), List.of(), Params.of("parsimony", 0.05),
                RobotSpec.DEFAULT);
    }

    @Test
    void aHandWrittenProgramSolvesTheEmptyFieldAndParsimonyPenalisesSize() {
        ControllerSynthesisProblem p = problem("empty-field");
        Program steer = p.parse(STEER);
        Evaluation e = p.evaluate(steer);
        assertEquals(1.0, e.metric("successRate", 0));
        Program padded = p.parse("(if (and goalAhead goalAhead) forward (if goalLeft turnLeft turnRight))");
        Evaluation f = p.evaluate(padded);
        assertEquals(e.metric("meanCost", 0), f.metric("meanCost", -1), 1e-12, "same behaviour");
        assertEquals(0.05 * (padded.size() - steer.size()), f.fitness() - e.fitness(), 1e-9);
        assertTrue(p.isTarget(e));
        Evaluation stop = p.evaluate(p.parse("stop"));
        assertTrue(stop.fitness() > 1000, "a program that never moves fails the episode");
    }

    @Test
    void theStepBudgetStopsOverlongInterpretation() {
        ControllerSynthesisProblem p = problem("empty-field");
        ProgramController c = new ProgramController(p.parse(STEER), 2);
        EpisodeResult r = p.simulate(p.training().get(0), c, EpisodeConfig.defaults().withMaxSteps(100));
        assertTrue(c.budgetExhausted() > 0);
        assertTrue(r.distance() < 0.5, "budget exhaustion means stop");
    }

    @Test
    void geneticProgrammingImprovesOnItsInitialPopulation() {
        ControllerSynthesisProblem p = problem("obstacle-course");
        SearchResult<Program> r = SearchAlgorithm.execute(AlgorithmRegistry.create("genetic-algorithm",
                Params.of("populationSize", 30)), SearchContext.builder(p, Budget.evaluations(600), 2).build());
        double initialBest = r.bestAt(30);
        assertTrue(r.best().fitness() < initialBest, initialBest + " -> " + r.best().fitness());
        assertTrue(r.best().solution().size() <= p.space().maxSize());
    }
}
