package org.robosbse.experiments;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.arm.ArmIkProblem;
import org.robosbse.arm.ArmMotion;
import org.robosbse.arm.ArmPickPlaceProblem;
import org.robosbse.arm.ArmScene;
import org.robosbse.arm.ArmTrajectoryProblem;
import org.robosbse.core.Problem;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.WorldMap;

/** Problem instances for the planar robotic arm (inverse kinematics, trajectories, pick and place). */
final class ArmInstances {

    private ArmInstances() {
    }

    /** Shared behaviour: the scene is inlined into results and replays carry it. */
    abstract static class Base implements ProblemInstance<RealVector> {
        final ArmScene scene;

        Base(ArmScene scene) {
            this.scene = scene;
        }

        /** The workspace as a map (width, height, obstacles), for generic bookkeeping only. */
        @Override
        public WorldMap map() {
            return new WorldMap(WorldMap.SCHEMA, scene.name(), scene.description(), scene.width(), scene.height(), null,
                    scene.target(), 0.01, scene.obstacles());
        }

        @Override
        public void normalise(ObjectNode problemNode) {
            problemNode.set("scene", Json.tree(scene));
        }

        @Override
        public List<Object> describeTargets(SearchResult<RealVector> result, Map<String, Object> provenance,
                                            boolean minimize, int limit) {
            return List.of();
        }

        @Override
        public RealVector parse(JsonNode genome) {
            return Problems.realGenome(genome);
        }

        Map<String, Object> withScene(Map<String, Object> replay) {
            Map<String, Object> out = new LinkedHashMap<>(replay);
            out.put("scene", scene);
            return out;
        }
    }

    static final class Ik extends Base {
        private final ArmIkProblem problem;

        Ik(ArmScene scene, ArmIkProblem problem) {
            super(scene);
            this.problem = problem;
        }

        @Override
        public Problem<RealVector> problem() {
            return problem;
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            double[] q = solution.toArray();
            Map<String, Object> out = new LinkedHashMap<>();
            // Inverse kinematics evaluates the final configuration only, so only that is shown (no motion).
            out.put("frames", ArmMotion.frames(problem.kinematics(), List.<double[]>of(q), null, 0.05));
            out.put("keyframes", List.<double[]>of(q));
            out.put("metrics", problem.evaluate(q).metrics());
            return withScene(out);
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }
    }

    static final class Trajectory extends Base {
        private final ArmTrajectoryProblem problem;

        Trajectory(ArmScene scene, ArmTrajectoryProblem problem) {
            super(scene);
            this.problem = problem;
        }

        @Override
        public Problem<RealVector> problem() {
            return problem;
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            return withScene(problem.replay(solution.toArray()));
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }
    }

    static final class PickPlace extends Base {
        private final ArmPickPlaceProblem problem;

        PickPlace(ArmScene scene, ArmPickPlaceProblem problem) {
            super(scene);
            this.problem = problem;
        }

        @Override
        public Problem<RealVector> problem() {
            return problem;
        }

        @Override
        public Map<String, Object> replay(RealVector solution) {
            return withScene(problem.replay(solution.toArray()));
        }

        @Override
        public Map<String, Object> reference() {
            return problem.reference();
        }
    }
}
