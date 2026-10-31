package org.robosbse.testing;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.Pose;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.controllers.ControllerSetup;
import org.robosbse.simulation.controllers.ReactiveAvoidanceController;
import org.robosbse.simulation.controllers.faulty.ReactiveBug3LowBatteryBoost;
import org.robosbse.testing.coverage.Branch;
import org.robosbse.testing.coverage.BranchFitness;
import org.robosbse.testing.coverage.CoverageAccumulator;
import org.robosbse.testing.coverage.CoverageTrace;
import org.robosbse.testing.coverage.Probe;

class CoverageTest {

    private static final List<Branch> TREE = List.of(
            Branch.root("a", "root"),
            Branch.under("b", "under a:true", "a", true),
            Branch.under("c", "under b:false", "b", false));

    @Test
    void probesReturnTheConditionAndRecordBranchDistances() {
        CoverageTrace trace = new CoverageTrace(TREE);
        Probe probe = new Probe();
        assertTrue(probe.lt(0, 1, 2)); // probe without trace still evaluates
        probe.attach(trace);
        assertFalse(probe.lt(0, 5, 2));
        assertEquals(3, trace.distance(0, true), 1e-6);
        assertTrue(probe.lt(0, 1.5, 2));
        assertTrue(trace.covered(0, true));
        assertTrue(trace.covered(0, false));
        assertTrue(probe.le(1, 2, 2));
        assertEquals(CoverageTrace.K, trace.distance(1, false), 1e-15);
        assertFalse(trace.reached(2));
        assertEquals(3, trace.coveredOutcomes());
    }

    @Test
    void branchFitnessUsesApproachLevelAndNormalisedDistance() {
        CoverageTrace trace = new CoverageTrace(TREE);
        Probe probe = new Probe();
        probe.attach(trace);
        probe.lt(0, 3, 2); // a evaluated, false, distance to true = 1
        BranchFitness.Value c = BranchFitness.of(trace, "c", true);
        assertEquals(2, c.approachLevel()); // c -> b (not reached) -> a (reached)
        assertEquals(2 + 1.0 / 2.0, c.fitness(), 1e-6);
        probe.lt(0, 1, 2); // a:true
        probe.flag(1, true); // b:true, so c is still not reached
        BranchFitness.Value c2 = BranchFitness.of(trace, "c", true);
        assertEquals(1, c2.approachLevel());
        assertEquals(1.5, c2.fitness(), 1e-9); // flag distance 1 -> 1/2
        probe.flag(1, false);
        probe.gt(2, 5, 1);
        assertTrue(BranchFitness.of(trace, "c", true).covered());
        assertEquals(0.0, BranchFitness.of(trace, "c", true).fitness());
    }

    private static EpisodeResult drive(ReactiveAvoidanceController controller, CoverageTrace trace, double battery) {
        RobotSpec robot = RobotSpec.DEFAULT;
        World world = new World(MapLibrary.builtin("obstacle-course"));
        Vec2 goal = new Vec2(17, 17);
        EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps(800).withAutoComplete(false)
                .withSensing(0.01, 0.2, 3).withBattery(battery);
        ControllerSetup setup = new ControllerSetup(robot, cfg.dt(), goal, 0.3, List.of(goal),
                Params.of("speedLimit", 0.9));
        if (trace != null) {
            controller.attach(trace);
        }
        return Simulator.run(world, robot, new Pose(2, 2, 0), goal, 0.3, controller, setup, cfg);
    }

    @Test
    void instrumentationDoesNotChangeBehaviour() {
        EpisodeResult plain = drive(new ReactiveAvoidanceController(), null, 1.0);
        CoverageTrace trace = new CoverageTrace(ReactiveAvoidanceController.BRANCHES);
        EpisodeResult traced = drive(new ReactiveAvoidanceController(), trace, 1.0);
        assertEquals(plain.steps(), traced.steps());
        assertEquals(plain.finalPose(), traced.finalPose());
        assertEquals(plain.energy(), traced.energy());
        assertArrayEquals(plain.pathSamples(), traced.pathSamples());
        assertTrue(trace.coveredOutcomes() >= 20, "a long drive with dropout covers most outcomes: " + trace.coveredOutcomes());
        assertTrue(trace.covered(trace.slot("scan-valid"), false), "20% dropout must lose scans");
    }

    @Test
    void lowBatteryBranchDistanceTracksTheBattery() {
        ReactiveBug3LowBatteryBoost bug3 = new ReactiveBug3LowBatteryBoost();
        CoverageTrace trace = new CoverageTrace(bug3.branches());
        EpisodeResult r = drive(bug3, trace, 0.5);
        int slot = trace.slot("low-battery-boost");
        double minBattery = 0.5 - r.energy() / RobotSpec.DEFAULT.batteryCapacity();
        assertFalse(trace.covered(slot, true));
        // The decision is evaluated each step before that step's energy is spent, so the smallest
        // observed battery is at most one step's consumption above the final charge.
        assertEquals(minBattery - 0.1, trace.distance(slot, true), 0.01);
        CoverageTrace low = new CoverageTrace(bug3.branches());
        drive(new ReactiveBug3LowBatteryBoost(), low, 0.09);
        assertTrue(low.covered(slot, true));
    }

    private static ControllerTestingProblem problem(String fitness, String target) {
        return new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                Params.of("controller", "reactive-bug3-boost", "properties", "safety", "fitness", fitness, "target", target),
                Params.empty(), RobotSpec.DEFAULT);
    }

    private static Scenario openDrive(double battery, double speed) {
        return new Scenario(new Pose(2, 6, 0), new Vec2(10, 6), 0, 0, battery, speed, 1.0, List.of());
    }

    @Test
    void branchFitnessGuidesFirstToTheBranchThenToTheViolation() {
        ControllerTestingProblem p = problem("branch:low-battery-boost:true", "violation");
        Evaluation far = p.evaluateScenario(openDrive(0.9, 1.0));
        Evaluation near = p.evaluateScenario(openDrive(0.3, 1.0));
        assertTrue(far.fitness() > near.fitness() && near.fitness() >= 1, far.fitness() + " vs " + near.fitness());
        assertEquals(0, near.metric("approachLevel", -1));
        Evaluation coveredNoFailure = p.evaluateScenario(openDrive(0.08, 0.6));
        assertEquals(1.0, coveredNoFailure.metric("targetCovered", 0));
        assertTrue(coveredNoFailure.fitness() > 0 && coveredNoFailure.fitness() < 1);
        Evaluation failure = p.evaluateScenario(openDrive(0.08, 1.0));
        assertTrue(failure.fitness() < 0);
        assertTrue(p.isTarget(failure));
        assertFalse(p.isTarget(coveredNoFailure));
        ControllerTestingProblem campaign = problem("branch:low-battery-boost:true", "branch");
        assertTrue(campaign.isTarget(campaign.evaluateScenario(openDrive(0.08, 0.6))));
    }

    @Test
    void invalidBranchSpecificationsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> problem("branch:no-such-decision:true", "violation"));
        assertThrows(IllegalArgumentException.class, () -> problem("branch:low-battery-boost:maybe", "violation"));
        assertThrows(IllegalArgumentException.class, () -> problem("min", "branch"));
    }

    @Test
    void accumulatorUnionsEpisodesAndKeepsTheSmallestWitness() {
        CoverageAccumulator acc = new CoverageAccumulator(TREE);
        CoverageTrace t1 = new CoverageTrace(TREE);
        t1.record(0, true, 0, 1);
        CoverageTrace t2 = new CoverageTrace(TREE);
        t2.record(0, true, 0, 1);
        t2.record(1, false, 1, 0);
        acc.add(t2, new double[] {2, 0});
        acc.add(t1, new double[] {1, 5});
        assertEquals(2, acc.coveredOutcomes());
        assertArrayEquals(new double[] {1, 5}, acc.witness("a", true));
        assertArrayEquals(new double[] {2, 0}, acc.witness("b", false));
        CoverageAccumulator other = new CoverageAccumulator(TREE);
        CoverageTrace t3 = new CoverageTrace(TREE);
        t3.record(2, true, 0, 1);
        other.add(t3, new double[] {0, 0});
        acc.merge(other);
        assertEquals(3, acc.coveredOutcomes());
        assertEquals(3, acc.episodes());
    }

    @Test
    void testingRunsReportCoverageOfAllGeneratedScenarios() {
        ControllerTestingProblem p = problem("min", "violation");
        assertNotNull(p.coverage());
        org.robosbse.core.Rng rng = new org.robosbse.core.Rng(4);
        for (int i = 0; i < 30; i++) {
            p.evaluate(p.space().random(rng));
        }
        assertTrue(p.coverage().episodes() > 0 && p.coverage().episodes() <= 30);
        assertTrue(p.coverage().coveredOutcomes() > 10);
        assertEquals(2 * p.branches().size(), p.coverage().totalOutcomes());
    }
}
