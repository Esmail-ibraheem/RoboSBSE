package org.robosbse.arm;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Params;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.geometry.Vec2;

class ArmTest {

    @Test
    void forwardKinematicsUsesRelativeJointAngles() {
        ArmKinematics kin = new ArmKinematics(ArmScene.builtin("arm-reach"));
        ArmScene s = kin.scene();
        Vec2[] p = kin.points(new double[] {Math.PI / 2, -Math.PI / 2});
        assertEquals(s.base().x(), p[1].x(), 1e-12);
        assertEquals(s.base().y() + s.links().get(0), p[1].y(), 1e-12);
        assertEquals(s.base().x() + s.links().get(1), p[2].x(), 1e-12);
        assertEquals(p[2], kin.endEffector(new double[] {Math.PI / 2, -Math.PI / 2}));
    }

    @Test
    void analyticalIkReachesTheTargetWithBothElbows() {
        for (String name : List.of("arm-reach", "arm-cluttered", "arm-pick-place")) {
            ArmKinematics kin = new ArmKinematics(ArmScene.builtin(name));
            Vec2 target = kin.scene().target();
            double[][] sol = kin.analyticalIk(target);
            for (double[] q : sol) {
                assertEquals(0, kin.endEffector(q).distanceTo(target), 1e-9, name);
            }
            assertTrue(Math.abs(sol[0][1] + sol[1][1]) < 1e-9, "the two elbows mirror each other");
        }
    }

    @Test
    void dampedLeastSquaresSolvesTheRedundantArm() {
        ArmKinematics kin = new ArmKinematics(ArmScene.builtin("arm-three-link"));
        ArmKinematics.Solve s = kin.numericalIk(kin.scene().target(), kin.scene().homeArray(), 300, 0.05);
        assertTrue(s.error() < 1e-3, "error " + s.error());
        assertEquals(s.error(), kin.endEffector(s.q()).distanceTo(kin.scene().target()), 1e-9);
    }

    @Test
    void clearanceTurnsNegativeWhenALinkCrossesAnObstacle() {
        ArmScene scene = ArmScene.builtin("arm-cluttered");
        ArmKinematics kin = new ArmKinematics(scene);
        assertTrue(kin.clearance(scene.homeArray()) > 0, "home pose is free");
        double minimum = Double.POSITIVE_INFINITY;
        double[] goal = kin.analyticalIk(scene.target())[1];
        for (double[] q : ArmKinematics.interpolate(scene.homeArray(), goal, 0.02)) {
            minimum = Math.min(minimum, kin.clearance(q));
        }
        assertTrue(minimum < 0, "the straight joint-space move sweeps through the post");
        assertTrue(kin.limitViolation(new double[] {-0.5, 0}) > 0.49);
    }

    @Test
    @SuppressWarnings("unchecked")
    void configurationSpacePlannerFindsTheCollisionFreeMotionTheStraightMoveMisses() {
        ArmScene scene = ArmScene.builtin("arm-cluttered");
        ArmKinematics kin = new ArmKinematics(scene);
        ConfigurationSpace cs = new ConfigurationSpace(kin, 0.02, 0.01);
        double[] goal = kin.analyticalIk(scene.target())[1];
        List<double[]> path = cs.plan(scene.homeArray(), goal);
        assertNotNull(path);
        assertArrayEquals(scene.homeArray(), path.get(0), 1e-12);
        assertArrayEquals(goal, path.get(path.size() - 1), 1e-12);
        for (int i = 1; i < path.size(); i++) {
            assertTrue(cs.straightFree(path.get(i - 1), path.get(i)), "shortcut segment " + i + " is free");
        }
        assertTrue(cs.expansions() > 0);
        ArmTrajectoryProblem p = new ArmTrajectoryProblem(scene, Params.empty());
        List<Map<String, Object>> options = (List<Map<String, Object>>) p.reference().get("straight");
        Map<String, Object> best = options.get(0);
        assertTrue(((String) best.get("name")).startsWith("C-space"), "planned motion ranks first: " + best.get("name"));
        assertEquals(1.0, ((Map<String, Double>) best.get("metrics")).get("feasible"));
        for (Map<String, Object> o : options) {
            if (((String) o.get("name")).startsWith("straight")) {
                assertEquals(0.0, ((Map<String, Double>) o.get("metrics")).get("feasible"), o.get("name").toString());
            }
        }
    }

    @Test
    void ikSearchMatchesTheAnalyticalSolution() {
        ArmScene scene = ArmScene.builtin("arm-reach");
        ArmIkProblem p = new ArmIkProblem(scene, Params.empty());
        SearchResult<RealVector> r = SearchAlgorithm.execute(AlgorithmRegistry.create("particle-swarm", Params.empty()),
                SearchContext.builder(p, Budget.evaluations(3000), 3).build());
        assertTrue(r.best().evaluation().metric("reached", 0) > 0, "PSO reaches the target");
        assertTrue(r.best().evaluation().metric("distance", 1) < 0.01);
    }

    @Test
    @SuppressWarnings("unchecked")
    void pickAndPlaceEncodingReplayAndReferencesAgree() {
        ArmScene scene = ArmScene.builtin("arm-pick-place");
        ArmPickPlaceProblem p = new ArmPickPlaceProblem(scene, Params.empty());
        ArmPickPlaceProblem.Plan greedy = p.greedy();
        ArmPickPlaceProblem.Plan decoded = p.decode(p.encode(greedy));
        assertArrayEquals(greedy.order(), decoded.order());
        assertArrayEquals(greedy.pickElbow(), decoded.pickElbow());
        assertArrayEquals(greedy.placeElbow(), decoded.placeElbow());
        Map<String, Object> reference = p.reference();
        double greedyFitness = (Double) ((Map<String, Object>) reference.get("greedy")).get("fitness");
        double optimum = (Double) ((Map<String, Object>) reference.get("optimum")).get("fitness");
        assertTrue(optimum <= greedyFitness + 1e-9);
        assertEquals(greedyFitness, p.evaluate(new RealVector((double[]) ((Map<String, Object>) reference.get("greedy"))
                .get("genome"))).fitness(), 1e-9, "the reference genome replays the greedy plan");

        Map<String, Object> replay = p.replay(p.encode(greedy));
        List<String> carrying = (List<String>) replay.get("carrying");
        assertEquals(((List<?>) replay.get("keyframes")).size(), carrying.size());
        List<String> carried = new ArrayList<>();
        for (String c : carrying) {
            if (c != null && (carried.isEmpty() || !carried.get(carried.size() - 1).equals(c))) {
                carried.add(c);
            }
        }
        List<String> expected = new ArrayList<>();
        for (int o : greedy.order()) {
            expected.add(scene.tasks().get(o).label());
        }
        assertEquals(expected, carried, "objects are carried one after another in plan order");

        SearchResult<RealVector> r = SearchAlgorithm.execute(AlgorithmRegistry.create("genetic-algorithm", Params.empty()),
                SearchContext.builder(p, Budget.evaluations(1500), 1).build());
        assertTrue(r.best().fitness() <= greedyFitness + 1e-9, "GA is at least as good as greedy");
        assertTrue(r.best().fitness() >= optimum - 1e-9, "nothing beats the enumerated optimum");
    }
}
