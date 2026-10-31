package org.robosbse.arm;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.Rng;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geometry.Vec2;

/**
 * Inverse kinematics (joint-angle optimisation) by search: find joint angles that put the end
 * effector on the target without collisions or joint-limit violations.
 *
 * <p>Genome: one angle per joint in [-pi, pi] (wider than the joint limits, which are penalised, not
 * enforced). Fitness: {@code |end - target| + weightCollision * max(0, -clearance) + weightLimits *
 * limitViolation + weightPosture * sum |q - home|}. Objectives: (distance, joint motion from home,
 * collision risk {@code exp(-clearance / 0.1 m)}). Target hit: distance below {@code tolerance},
 * no collision, within limits. References: the analytical solution for two-link arms (both elbows) and
 * damped-least-squares IK from several starts for any arm.
 */
public final class ArmIkProblem implements Problem<RealVector> {

    public static final String TYPE = "arm-ik";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.real("weightCollision", 10, 0, 1e6, "Penalty per metre of link penetration."),
            ParamSpec.real("weightLimits", 1, 0, 1e6, "Penalty per radian of joint-limit violation."),
            ParamSpec.real("weightPosture", 0, 0, 1e6, "Weight of the joint motion away from the home pose."),
            ParamSpec.real("tolerance", 0.01, 1e-5, 1, "Distance at which the target counts as reached (m)."));

    private final ArmKinematics kin;
    private final RealVectorSpace space;
    private final Vec2 target;
    private final double weightCollision;
    private final double weightLimits;
    private final double weightPosture;
    private final double tolerance;

    public ArmIkProblem(ArmScene scene, Params params) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid arm-ik parameters: " + String.join("; ", errors));
        }
        this.kin = new ArmKinematics(scene);
        this.target = scene.target();
        this.space = angleSpace(scene.joints(), 1, "q");
        this.weightCollision = params.getDouble("weightCollision", 10);
        this.weightLimits = params.getDouble("weightLimits", 1);
        this.weightPosture = params.getDouble("weightPosture", 0);
        this.tolerance = params.getDouble("tolerance", 0.01);
    }

    static RealVectorSpace angleSpace(int joints, int blocks, String prefix) {
        int n = joints * blocks;
        double[] lo = new double[n];
        double[] hi = new double[n];
        List<String> names = new ArrayList<>();
        for (int b = 0; b < blocks; b++) {
            for (int j = 0; j < joints; j++) {
                lo[b * joints + j] = -Math.PI;
                hi[b * joints + j] = Math.PI;
                names.add(blocks == 1 ? prefix + j : prefix + b + "." + j);
            }
        }
        return new RealVectorSpace(lo, hi, names, joints);
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    public ArmKinematics kinematics() {
        return kin;
    }

    public Vec2 target() {
        return target;
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluate(genome.toArray());
    }

    public Evaluation evaluate(double[] q) {
        Vec2 end = kin.endEffector(q);
        double distance = end.distanceTo(target);
        double clearance = kin.clearance(q);
        double limits = kin.limitViolation(q);
        double[] home = kin.scene().homeArray();
        double motion = 0;
        for (int i = 0; i < q.length; i++) {
            motion += Math.abs(q[i] - home[i]);
        }
        double penetration = Math.max(0, -clearance);
        double fitness = distance + weightCollision * penetration + weightLimits * limits + weightPosture * motion;
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("distance", distance);
        m.put("clearance", Math.min(clearance, 10));
        m.put("limitViolation", limits);
        m.put("jointMotion", motion);
        m.put("reached", distance < tolerance && clearance >= 0 && limits == 0 ? 1.0 : 0.0);
        double risk = Math.exp(-Math.max(0, clearance) / 0.1);
        ArmScene s = kin.scene();
        double[] behaviour = {(end.x() - (s.base().x() - s.reach())) / (2 * s.reach()), (end.y() - (s.base().y() - s.reach())) / (2 * s.reach())};
        return new Evaluation(fitness, penetration + limits + Math.max(0, distance - tolerance), m,
                new double[] {distance, motion, risk}, behaviour);
    }

    @Override
    public boolean isTarget(Evaluation e) {
        return e.metric("reached", 0) > 0;
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("distance", "jointMotion", "risk");
    }

    @Override
    public double[] referencePoint() {
        return new double[] {2 * kin.scene().reach(), 2 * Math.PI * kin.joints(), 1.0};
    }

    @Override
    public Object view(RealVector genome) {
        double[] q = genome.toArray();
        Map<String, Object> v = new LinkedHashMap<>();
        v.put("q", q);
        List<double[]> points = new ArrayList<>();
        for (Vec2 p : kin.points(q)) {
            points.add(new double[] {p.x(), p.y()});
        }
        v.put("points", points);
        v.put("genome", q);
        return v;
    }

    /** Analytical (two-link) and numerical reference solutions, each with its evaluation. */
    public Map<String, Object> reference() {
        Map<String, Object> out = new LinkedHashMap<>();
        if (kin.joints() == 2) {
            List<Map<String, Object>> analytical = new ArrayList<>();
            String[] names = {"elbow +", "elbow -"};
            double[][] sol = kin.analyticalIk(target);
            for (int i = 0; i < 2; i++) {
                analytical.add(describe(names[i], sol[i], 0));
            }
            out.put("analytical", analytical);
        }
        Rng rng = new Rng(17);
        List<Map<String, Object>> numerical = new ArrayList<>();
        int iterations = 0;
        for (int start = 0; start < 8; start++) {
            double[] q0 = start == 0 ? kin.scene().homeArray() : space.random(rng).toArray();
            ArmKinematics.Solve s = kin.numericalIk(target, q0, 300, 0.05);
            iterations += s.iterations();
            numerical.add(describe("DLS start " + start, s.q(), s.iterations()));
        }
        numerical.sort((a, b) -> Double.compare((Double) a.get("fitness"), (Double) b.get("fitness")));
        out.put("numerical", numerical.subList(0, Math.min(3, numerical.size())));
        out.put("numericalIterations", iterations);
        out.put("note", "Analytical IK (two-link arms) and damped least squares from 8 starts; iterations are not fitness evaluations.");
        return out;
    }

    private Map<String, Object> describe(String name, double[] q, int iterations) {
        Evaluation e = evaluate(q);
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("name", name);
        m.put("q", q);
        m.put("fitness", e.fitness());
        m.put("metrics", e.metrics());
        if (iterations > 0) {
            m.put("iterations", iterations);
        }
        return m;
    }
}
