package org.robosbse.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.robosbse.core.Evaluation;
import org.robosbse.core.ParamSpec;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.geometry.Vec2;

/**
 * Pick-and-place task planning for a two-link arm: in which order to move the objects, and with which
 * elbow configuration to pick up and put down each one (task planning), with collision-free motions
 * between the chosen poses supplied by the {@link ConfigurationSpace} planner (motion planning).
 *
 * <p><b>Encoding: random keys</b> (Bean, 1994), so that every real-vector algorithm (PSO, GA, hill
 * climbing, SA, NSGA-II) applies: {@code 3K} genes in [0, 1] for K objects, namely K priorities (the
 * objects are handled in increasing priority) and K + K elbow genes for the pick and the place pose
 * (elbow + when the gene is at least 0.5). The motion is home, then pick and place of every object in
 * order, then home, with a handling pause at every pick and place. Each move between two poses is the
 * planner's time-optimal collision-free path, computed once per pair of poses and cached. A pose that is
 * out of reach, outside the joint limits or in collision cannot be planned; the move is then a straight
 * joint-space line (measured, collisions included) and the pose is penalised.
 *
 * <p><b>Fitness</b>: total time + 100 per invalid pose + {@code weightCollision * penetration} +
 * {@code weightEffort * effort}. Objectives: (time, effort, collision risk). <b>References</b>: greedy
 * (repeatedly the cheapest next object and elbows) and, for up to five objects, the exact optimum of this
 * model by enumerating every order and elbow choice.
 */
public final class ArmPickPlaceProblem implements Problem<RealVector> {

    public static final String TYPE = "arm-pick-place";

    public static final List<ParamSpec> PARAMS = List.of(
            ParamSpec.real("handlingTime", 1.0, 0, 100, "Pause at every pick and place (s)."),
            ParamSpec.real("weightCollision", 50, 0, 1e6, "Weight of the penetration integral (fallback moves only)."),
            ParamSpec.real("weightEffort", 0, 0, 1e6, "Weight of the effort (energy) model."));

    private final ArmKinematics kin;
    private final RealVectorSpace space;
    private final int k;
    private final double handling;
    private final double wCollision;
    private final double wEffort;
    /** Pose 0 is home; pose 1 + 4o + 2w + e is object o, w = 0 pick / 1 place, elbow e = 0 (+) / 1 (-). */
    private final List<double[]> poses = new ArrayList<>();
    private final boolean[] valid;
    private final ConfigurationSpace planner;
    private final Map<Long, Move> moves = new ConcurrentHashMap<>();

    /** A planned (or fallback) move between two poses and its measurements. */
    record Move(List<double[]> keyframes, ArmMotion.Stats stats, boolean planned) {
    }

    public ArmPickPlaceProblem(ArmScene scene, Params params) {
        List<String> errors = ParamSpec.validate(PARAMS, params);
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("invalid arm-pick-place parameters: " + String.join("; ", errors));
        }
        if (scene.joints() != 2) {
            throw new IllegalArgumentException("pick-and-place planning uses analytical IK and needs a two-link arm");
        }
        if (scene.tasks().isEmpty()) {
            throw new IllegalArgumentException("scene " + scene.name() + " has no pick-and-place tasks");
        }
        this.kin = new ArmKinematics(scene);
        this.k = scene.tasks().size();
        this.handling = params.getDouble("handlingTime", 1.0);
        this.wCollision = params.getDouble("weightCollision", 50);
        this.wEffort = params.getDouble("weightEffort", 0);
        double[] lo = new double[3 * k];
        double[] hi = new double[3 * k];
        List<String> names = new ArrayList<>();
        for (int i = 0; i < 3 * k; i++) {
            hi[i] = 1;
            String label = scene.tasks().get(i % k).label();
            names.add((i < k ? "priority " : i < 2 * k ? "pick elbow " : "place elbow ") + label);
        }
        this.space = new RealVectorSpace(lo, hi, names, 1);
        poses.add(scene.homeArray());
        for (ArmScene.Task t : scene.tasks()) {
            for (Vec2 p : new Vec2[] {t.pick(), t.place()}) {
                double[][] sol = kin.analyticalIk(p);
                poses.add(sol[0]);
                poses.add(sol[1]);
            }
        }
        this.valid = new boolean[poses.size()];
        for (int i = 0; i < poses.size(); i++) {
            double[] q = poses.get(i);
            Vec2 want = i == 0 ? kin.endEffector(q) : point(i);
            valid[i] = kin.endEffector(q).distanceTo(want) < 1e-3 && kin.limitViolation(q) == 0 && kin.clearance(q) >= 0;
        }
        this.planner = new ConfigurationSpace(kin, 0.02, 0.01);
    }

    private Vec2 point(int pose) {
        int o = (pose - 1) / 4;
        int w = ((pose - 1) / 2) % 2;
        ArmScene.Task t = kin.scene().tasks().get(o);
        return w == 0 ? t.pick() : t.place();
    }

    private static int poseIndex(int object, int which, int elbow) {
        return 1 + 4 * object + 2 * which + elbow;
    }

    /** The move from pose a to pose b: planned when both are valid and connected, else a straight line. */
    Move move(int a, int b) {
        return moves.computeIfAbsent(((long) a << 32) | b, key -> {
            List<double[]> planned = valid[a] && valid[b] ? planner.plan(poses.get(a), poses.get(b)) : null;
            List<double[]> keys = planned != null ? planned : List.of(poses.get(a), poses.get(b));
            return new Move(keys, ArmMotion.measure(kin, keys, null), planned != null);
        });
    }

    @Override
    public String type() {
        return TYPE;
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    /** A decoded plan: object order and elbow choices. */
    public record Plan(int[] order, int[] pickElbow, int[] placeElbow) {
    }

    public Plan decode(double[] g) {
        Integer[] idx = new Integer[k];
        for (int i = 0; i < k; i++) {
            idx[i] = i;
        }
        Arrays.sort(idx, Comparator.comparingDouble((Integer i) -> g[i]).thenComparingInt(i -> i));
        int[] order = new int[k];
        int[] pick = new int[k];
        int[] place = new int[k];
        for (int i = 0; i < k; i++) {
            order[i] = idx[i];
            pick[i] = g[k + i] >= 0.5 ? 0 : 1;
            place[i] = g[2 * k + i] >= 0.5 ? 0 : 1;
        }
        return new Plan(order, pick, place);
    }

    /** A genome that decodes to the given plan (for replaying reference plans). */
    public double[] encode(Plan plan) {
        double[] g = new double[3 * k];
        for (int i = 0; i < k; i++) {
            int o = plan.order()[i];
            g[o] = (i + 0.5) / k;
            g[k + o] = plan.pickElbow()[o] == 0 ? 0.75 : 0.25;
            g[2 * k + o] = plan.placeElbow()[o] == 0 ? 0.75 : 0.25;
        }
        return g;
    }

    /** Pose indices visited by a plan: home, pick and place of every object, home. */
    int[] route(Plan plan) {
        int[] r = new int[2 * k + 2];
        int n = 1;
        for (int o : plan.order()) {
            r[n++] = poseIndex(o, 0, plan.pickElbow()[o]);
            r[n++] = poseIndex(o, 1, plan.placeElbow()[o]);
        }
        return r;
    }

    @Override
    public Evaluation evaluate(RealVector genome) {
        return evaluate(decode(genome.toArray()));
    }

    public Evaluation evaluate(Plan plan) {
        int[] r = route(plan);
        double time = 2 * k * handling;
        double travel = 0;
        double effort = 0;
        double roughness = 0;
        double penetration = 0;
        double minClearance = Double.POSITIVE_INFINITY;
        int colliding = 0;
        int unplanned = 0;
        for (int i = 1; i < r.length; i++) {
            Move m = move(r[i - 1], r[i]);
            ArmMotion.Stats s = m.stats();
            time += s.time();
            travel += s.travel();
            effort += s.effort();
            roughness += s.roughness();
            penetration += s.penetration();
            minClearance = Math.min(minClearance, s.minClearance());
            colliding += s.collidingSamples();
            unplanned += m.planned() ? 0 : 1;
        }
        int invalid = 0;
        for (int i = 1; i < r.length - 1; i++) {
            invalid += valid[r[i]] ? 0 : 1;
        }
        double fitness = time + 100 * invalid + wCollision * penetration + wEffort * effort;
        Map<String, Double> m = new LinkedHashMap<>();
        m.put("time", time);
        m.put("travel", travel);
        m.put("effort", effort);
        m.put("roughness", roughness);
        m.put("penetration", penetration);
        m.put("minClearance", Math.min(minClearance, 10));
        m.put("invalidPoses", (double) invalid);
        m.put("unplannedMoves", (double) unplanned);
        boolean feasible = invalid == 0 && colliding == 0 && unplanned == 0;
        m.put("feasible", feasible ? 1.0 : 0.0);
        double risk = Math.exp(-Math.max(0, minClearance) / 0.1);
        double first = plan.order()[0] / (double) Math.max(1, k - 1);
        double elbows = 0;
        for (int i = 0; i < k; i++) {
            elbows += plan.pickElbow()[i] + plan.placeElbow()[i];
        }
        return new Evaluation(fitness, invalid + unplanned + penetration, m, new double[] {time, effort, risk},
                new double[] {first, elbows / (2.0 * k)});
    }

    @Override
    public boolean isTarget(Evaluation e) {
        return e.metric("feasible", 0) > 0;
    }

    @Override
    public List<String> objectiveNames() {
        return List.of("time", "effort", "risk");
    }

    @Override
    public double[] referencePoint() {
        double maxTime = (2 * k + 1) * 2 * Math.PI / kin.scene().maxJointSpeed() + 2 * k * handling;
        return new double[] {maxTime, 100, 1.0};
    }

    @Override
    public Object view(RealVector genome) {
        return describe(decode(genome.toArray()), genome.toArray());
    }

    Map<String, Object> describe(Plan plan, double[] genome) {
        Map<String, Object> v = new LinkedHashMap<>();
        List<String> steps = new ArrayList<>();
        for (int o : plan.order()) {
            ArmScene.Task t = kin.scene().tasks().get(o);
            steps.add(t.label() + " (pick elbow " + (plan.pickElbow()[o] == 0 ? "+" : "-") + ", place elbow "
                    + (plan.placeElbow()[o] == 0 ? "+" : "-") + ")");
        }
        v.put("plan", steps);
        v.put("order", plan.order());
        if (genome != null) {
            v.put("genome", genome);
        }
        return v;
    }

    /** Greedy plan: from the current pose, repeatedly take the object and elbows with the cheapest next pick and place. */
    public Plan greedy() {
        boolean[] done = new boolean[k];
        int[] order = new int[k];
        int[] pick = new int[k];
        int[] place = new int[k];
        int current = 0;
        for (int step = 0; step < k; step++) {
            double best = Double.POSITIVE_INFINITY;
            int bo = -1;
            int bp = 0;
            int bq = 0;
            for (int o = 0; o < k; o++) {
                if (done[o]) {
                    continue;
                }
                for (int e1 = 0; e1 < 2; e1++) {
                    for (int e2 = 0; e2 < 2; e2++) {
                        int p = poseIndex(o, 0, e1);
                        int q = poseIndex(o, 1, e2);
                        Move m1 = move(current, p);
                        Move m2 = move(p, q);
                        double cost = m1.stats().time() + m2.stats().time()
                                + 100 * ((valid[p] ? 0 : 1) + (valid[q] ? 0 : 1))
                                + wCollision * (m1.stats().penetration() + m2.stats().penetration());
                        if (cost < best) {
                            best = cost;
                            bo = o;
                            bp = e1;
                            bq = e2;
                        }
                    }
                }
            }
            done[bo] = true;
            order[step] = bo;
            pick[bo] = bp;
            place[bo] = bq;
            current = poseIndex(bo, 1, bq);
        }
        return new Plan(order, pick, place);
    }

    /** Exact optimum of this model by enumeration (K! x 4^K plans); null for more than five objects. */
    public Map<String, Object> optimum() {
        if (k > 5) {
            return null;
        }
        int[] order = new int[k];
        for (int i = 0; i < k; i++) {
            order[i] = i;
        }
        double best = Double.POSITIVE_INFINITY;
        Plan bestPlan = null;
        long plans = 0;
        do {
            for (int mask = 0; mask < (1 << (2 * k)); mask++) {
                int[] pick = new int[k];
                int[] place = new int[k];
                for (int o = 0; o < k; o++) {
                    pick[o] = (mask >> o) & 1;
                    place[o] = (mask >> (k + o)) & 1;
                }
                Plan p = new Plan(order.clone(), pick, place);
                double f = evaluate(p).fitness();
                plans++;
                if (f < best) {
                    best = f;
                    bestPlan = p;
                }
            }
        } while (nextPermutation(order));
        Map<String, Object> out = new LinkedHashMap<>(describe(bestPlan, encode(bestPlan)));
        out.put("fitness", best);
        out.put("metrics", evaluate(bestPlan).metrics());
        out.put("plansEnumerated", plans);
        return out;
    }

    private static boolean nextPermutation(int[] a) {
        int i = a.length - 2;
        while (i >= 0 && a[i] >= a[i + 1]) {
            i--;
        }
        if (i < 0) {
            return false;
        }
        int j = a.length - 1;
        while (a[j] <= a[i]) {
            j--;
        }
        int t = a[i];
        a[i] = a[j];
        a[j] = t;
        for (int l = i + 1, r = a.length - 1; l < r; l++, r--) {
            t = a[l];
            a[l] = a[r];
            a[r] = t;
        }
        return true;
    }

    public Map<String, Object> reference() {
        Map<String, Object> out = new LinkedHashMap<>();
        Plan g = greedy();
        Map<String, Object> greedy = new LinkedHashMap<>(describe(g, encode(g)));
        greedy.put("fitness", evaluate(g).fitness());
        greedy.put("metrics", evaluate(g).metrics());
        out.put("greedy", greedy);
        Map<String, Object> opt = optimum();
        if (opt != null) {
            out.put("optimum", opt);
        }
        out.put("plannerExpansions", planner.expansions());
        out.put("note", "Moves between poses are planned in configuration space (A*, 0.02 rad grid). Greedy: cheapest next "
                + "object and elbows. Optimum: every order and elbow choice enumerated (exact for this model).");
        return out;
    }

    public Map<String, Object> replay(double[] genome) {
        Plan plan = decode(genome);
        int[] r = route(plan);
        List<double[]> keys = new ArrayList<>();
        List<Double> pauses = new ArrayList<>();
        List<String> carrying = new ArrayList<>();
        keys.add(poses.get(0));
        pauses.add(0.0);
        carrying.add(null);
        for (int i = 1; i < r.length; i++) {
            List<double[]> seg = move(r[i - 1], r[i]).keyframes();
            // Even moves (2, 4, ..., 2K) go from an object's pick pose to its place pose.
            String carried = i % 2 == 0 && i <= 2 * k ? kin.scene().tasks().get(plan.order()[i / 2 - 1]).label() : null;
            for (int j = 1; j < seg.size(); j++) {
                keys.add(seg.get(j));
                boolean stop = j == seg.size() - 1 && i < r.length - 1;
                pauses.add(stop ? handling : 0.0);
                carrying.add(carried);
            }
        }
        double[] p = pauses.stream().mapToDouble(Double::doubleValue).toArray();
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("frames", ArmMotion.frames(kin, keys, p, 0.05));
        out.put("keyframes", keys);
        out.put("carrying", carrying);
        out.put("plan", describe(plan, null).get("plan"));
        out.put("metrics", evaluate(plan).metrics());
        return out;
    }
}
