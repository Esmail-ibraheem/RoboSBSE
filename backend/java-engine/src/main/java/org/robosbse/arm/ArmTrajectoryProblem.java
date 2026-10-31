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
 * Trajectory optimisation: move the arm from its home configuration to a configuration that reaches
 * the target, through {@code viaPoints} intermediate configurations, without collisions, smoothly and
 * cheaply (collision-free, energy-efficient manipulation).
 *
 * <p>Genome: {@code viaPoints + 1} configurations (the last one is the final pose), each angle in
 * [-pi, pi]. The motion between keyframes is measured by {@link ArmMotion}. Fitness:
 * {@code wR * reach + wC * penetration + wL * limits + wT * travel + wE * effort + wS * roughness},
 * where reach is the final end-effector distance to the target. Feasible = reach below
 * {@code tolerance}, no collision, within the limits. Objectives for NSGA-II: (time, effort,
 * roughness), with the infeasibility as constraint violation. Reference: the straight joint-space move
 * from home to each IK solution of the target (analytical for two links, damped least squares otherwise).
 */
public final class ArmTrajectoryProblem implements Problem<RealVector> {

    public static final String TYPE = "arm-trajectory";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.integer("viaPoints", 3, 0, 20, "Intermediate configurations between home and the final pose."),
            ParamSpec.real("weightReach", 100, 0, 1e6, "Weight of the final distance to the target (per metre)."),
            ParamSpec.real("weightCollision", 50, 0, 1e6, "Weight of the penetration integral (m x rad)."),
            ParamSpec.real("weightLimits", 10, 0, 1e6, "Weight of joint-limit violations (rad)."),
            ParamSpec.real("weightTravel", 1, 0, 1e6, "Weight of the total joint travel (rad): movement cost."),
            ParamSpec.real("weightEffort", 0, 0, 1e6, "Weight of the effort (energy) model."),
            ParamSpec.real("weightSmooth", 0.1, 0, 1e6, "Weight of the roughness (joint-space direction changes)."),
            ParamSpec.real("tolerance", 0.02, 1e-5, 1, "Distance at which the target counts as reached (m)."));

    private final ArmKinematics kin;
    private final RealVectorSpace space;
    private final int via;
    private final double wReach;
    private final double wCollision;
    private final double wLimits;
    private final double wTravel;
    private final double wEffort;
    private final double wSmooth;
    private final double tolerance;

    public ArmTrajectoryProblem(ArmScene scene, Params params) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid arm-trajectory parameters: " + String.join("; ", errors));
        }
        this.kin = new ArmKinematics(scene);
        this.via = params.getInt("viaPoints", 3);
        this.space = ArmIkProblem.angleSpace(scene.joints(), via + 1, "k");
        this.wReach = params.getDouble("weightReach", 100);
        this.wCollision = params.getDouble("weightCollision", 50);
        this.wLimits = params.getDouble("weightLimits", 10);
        this.wTravel = params.getDouble("weightTravel", 1);
        this.wEffort = params.getDouble("weightEffort", 0);
        this.wSmooth = params.getDouble("weightSmooth", 0.1);
        this.tolerance = params.getDouble("tolerance", 0.02);
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

    /** Home plus the genome's configurations. */
    public List<double[]> keyframes(double[] genome) {
        int n = kin.joints();
        List<double[]> out = new ArrayList<>();
        out.add(kin.scene().homeArray());
        for (int k = 0; k <= via; k++) {
            double[] q = new double[n];
            System.arraycopy(genome, k * n, q, 0, n);
            out.add(q);
        }
        return out;
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluateKeyframes(keyframes(genome.toArray()));
    }

    public Evaluation evaluateKeyframes(List<double[]> keyframes) {
        ArmMotion.Stats s = ArmMotion.measure(kin, keyframes, null);
        Vec2 end = kin.endEffector(keyframes.get(keyframes.size() - 1));
        double reach = end.distanceTo(kin.scene().target());
        double fitness = wReach * reach + wCollision * s.penetration() + wLimits * s.limits() + wTravel * s.travel()
                + wEffort * s.effort() + wSmooth * s.roughness();
        Map<String, Double> m = new LinkedHashMap<>(s.metrics());
        m.put("reach", reach);
        boolean feasible = reach < tolerance && s.collisionFree() && s.limits() == 0;
        m.put("feasible", feasible ? 1.0 : 0.0);
        double violation = Math.max(0, reach - tolerance) + s.penetration() + s.limits() + (s.collisionFree() ? 0 : 0.01);
        ArmScene sc = kin.scene();
        double[] behaviour = {(end.x() - (sc.base().x() - sc.reach())) / (2 * sc.reach()),
                Math.min(1, s.travel() / (Math.PI * kin.joints()))};
        return new Evaluation(fitness, violation, m, new double[] {s.time(), s.effort(), s.roughness()}, behaviour);
    }

    @Override
    public boolean isTarget(Evaluation e) {
        return e.metric("feasible", 0) > 0;
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("time", "effort", "roughness");
    }

    @Override
    public double[] referencePoint() {
        double maxTime = 2 * Math.PI * (via + 1) / kin.scene().maxJointSpeed();
        return new double[] {maxTime, 50, 2 * Math.PI * kin.joints()};
    }

    @Override
    public Object view(RealVector genome) {
        Map<String, Object> v = new LinkedHashMap<>();
        v.put("keyframes", keyframes(genome.toArray()));
        v.put("genome", genome.toArray());
        return v;
    }

    /** The genome of a straight joint-space move from home to {@code goal} (via points on the line). */
    public double[] straightGenome(double[] goal) {
        int n = kin.joints();
        double[] home = kin.scene().homeArray();
        double[] g = new double[(via + 1) * n];
        for (int k = 0; k <= via; k++) {
            double f = (double) (k + 1) / (via + 1);
            for (int i = 0; i < n; i++) {
                g[k * n + i] = home[i] + f * (goal[i] - home[i]);
            }
        }
        return g;
    }

    /** Straight moves to the IK solutions of the target, evaluated with the same fitness. */
    public Map<String, Object> reference() {
        List<Map<String, Object>> options = new ArrayList<>();
        long expansions = 0;
        if (kin.joints() == 2) {
            double[][] sol = kin.analyticalIk(kin.scene().target());
            options.add(option("straight move to elbow + IK", sol[0]));
            options.add(option("straight move to elbow - IK", sol[1]));
            ConfigurationSpace cs = new ConfigurationSpace(kin, 0.02, 0.01);
            String[] names = {"C-space A* to elbow + IK", "C-space A* to elbow - IK"};
            for (int i = 0; i < 2; i++) {
                List<double[]> planned = cs.plan(kin.scene().homeArray(), sol[i]);
                if (planned != null) {
                    Evaluation e = evaluateKeyframes(planned);
                    Map<String, Object> m = new LinkedHashMap<>();
                    m.put("name", names[i]);
                    m.put("fitness", e.fitness());
                    m.put("metrics", e.metrics());
                    m.put("keyframes", planned);
                    m.put("frames", ArmMotion.frames(kin, planned, null, 0.05));
                    options.add(m);
                }
            }
            expansions = cs.expansions();
        } else {
            Rng rng = new Rng(17);
            for (int start = 0; start < 4; start++) {
                double[] q0 = start == 0 ? kin.scene().homeArray() : ArmIkProblem.angleSpace(kin.joints(), 1, "q").random(rng).toArray();
                double[] q = kin.numericalIk(kin.scene().target(), q0, 300, 0.05).q();
                options.add(option("straight move to DLS IK (start " + start + ")", q));
            }
        }
        options.sort((a, b) -> Double.compare((Double) a.get("fitness"), (Double) b.get("fitness")));
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("straight", options);
        out.put("plannerExpansions", expansions);
        out.put("note", "Conventional references: the straight joint-space move to an IK solution, and (two-link arms) the "
                + "time-optimal collision-free grid path from the configuration-space planner (A*, 0.02 rad, shortcut).");
        return out;
    }

    private Map<String, Object> option(String name, double[] goal) {
        double[] g = straightGenome(goal);
        Evaluation e = evaluate(new RealVector(g));
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("name", name);
        m.put("fitness", e.fitness());
        m.put("metrics", e.metrics());
        m.put("genome", g);
        return m;
    }

    /** Recorded motion for replays. */
    public Map<String, Object> replay(double[] genome) {
        List<double[]> keys = keyframes(genome);
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("frames", ArmMotion.frames(kin, keys, null, 0.05));
        out.put("keyframes", keys);
        out.put("metrics", evaluateKeyframes(keys).metrics());
        return out;
    }
}
