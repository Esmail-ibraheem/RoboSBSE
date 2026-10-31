package org.robosbse.geneticprogramming;

import java.util.ArrayList;
import java.util.List;

import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SolutionSpace;

/**
 * Representation of controller programs for every generic algorithm (GA, hill climbing, SA, novelty
 * search, ...).
 *
 * <ul>
 *   <li><b>random</b>: ramped half-and-half (Koza): depth uniform in [{@code minInitDepth},
 *       {@code maxInitDepth}], half "full" trees and half "grow" trees, retried until within the
 *       limits.</li>
 *   <li><b>mutation</b> {@code subtree}: a random node is replaced by a newly grown subtree of the same
 *       type (depth up to 4); {@code point}: a node is replaced by another primitive with the same
 *       signature, or a constant is perturbed (sigma 0.5); {@code constant}: every constant is perturbed
 *       (sigma 0.3); {@code mixed} (default): subtree 50%, point 35%, constant 15%.</li>
 *   <li><b>crossover</b> {@code subtree}: swaps two subtrees of the same type, choosing internal nodes
 *       with probability 0.9.</li>
 *   <li><b>neighbourhood</b>: all point mutations and constant steps of +-0.25 and +-1 (enumerable),
 *       random = a point mutation.</li>
 * </ul>
 * Bloat control (Luke and Panait): every operator respects {@code maxSize} and {@code maxDepth}; an
 * offspring that would exceed them is discarded and the parent is returned (crossover) or another
 * mutation is drawn. Parsimony pressure is applied by the problem's fitness. Distance = normalised
 * Levenshtein distance between the preorder token lists.
 */
public final class ProgramSpace implements SolutionSpace<Program> {

    public static final List<String> MUTATIONS = List.of("mixed", "subtree", "point", "constant");
    public static final List<String> CROSSOVERS = List.of("subtree");

    private final int maxSize;
    private final int maxDepth;
    private final int minInitDepth;
    private final int maxInitDepth;

    public ProgramSpace(int maxSize, int maxDepth, int minInitDepth, int maxInitDepth) {
        if (maxSize < 1 || maxDepth < 0 || minInitDepth < 1 || maxInitDepth < minInitDepth || maxInitDepth > maxDepth) {
            throw new IllegalArgumentException("invalid program limits");
        }
        this.maxSize = maxSize;
        this.maxDepth = maxDepth;
        this.minInitDepth = minInitDepth;
        this.maxInitDepth = maxInitDepth;
    }

    public int maxSize() {
        return maxSize;
    }

    public int maxDepth() {
        return maxDepth;
    }

    public boolean withinLimits(Node n) {
        return n.size() <= maxSize && n.depth() <= maxDepth;
    }

    @Override
    public String representation() {
        return "program";
    }

    @Override
    public Program random(Rng rng) {
        for (int attempt = 0; attempt < 100; attempt++) {
            int depth = minInitDepth + rng.nextInt(maxInitDepth - minInitDepth + 1);
            Node n = generate(Op.Type.ACTION, depth, rng.chance(0.5), rng);
            if (withinLimits(n)) {
                return new Program(n);
            }
        }
        return new Program(generate(Op.Type.ACTION, minInitDepth, false, rng));
    }

    /** Full (every branch reaches {@code depth}) or grow (terminals may appear earlier) generation. */
    static Node generate(Op.Type type, int depth, boolean full, Rng rng) {
        List<Op> terminals = Op.of(type, true);
        List<Op> functions = Op.of(type, false);
        boolean terminal = depth <= 0 || functions.isEmpty()
                || (!full && rng.chance((double) terminals.size() / (terminals.size() + functions.size())));
        if (terminal) {
            Op op = terminals.get(rng.nextInt(terminals.size()));
            return op == Op.CONST ? randomConstant(rng) : Node.of(op);
        }
        Op op = functions.get(rng.nextInt(functions.size()));
        List<Node> children = new ArrayList<>();
        for (Op.Type t : op.args()) {
            children.add(generate(t, depth - 1, full, rng));
        }
        return Node.of(op, children);
    }

    /** Ephemeral random constant in [-1, 3] (distances in metres, bearings in radians). */
    static Node randomConstant(Rng rng) {
        return Node.constant(rng.uniform(-1, 3));
    }

    @Override
    public MutationOperator<Program> mutation(Params params) {
        String name = SolutionSpace.operator(params, "mutation", MUTATIONS, representation());
        return (parent, rng) -> {
            for (int attempt = 0; attempt < 20; attempt++) {
                String kind = name;
                if (kind.equals("mixed")) {
                    double u = rng.nextDouble();
                    kind = u < 0.5 ? "subtree" : u < 0.85 ? "point" : "constant";
                }
                Node child = switch (kind) {
                    case "subtree" -> subtreeMutation(parent.root(), rng);
                    case "point" -> pointMutation(parent.root(), rng);
                    default -> constantMutation(parent.root(), 0.3, rng);
                };
                if (child != null && withinLimits(child) && !child.equals(parent.root())) {
                    return new Program(child);
                }
            }
            return new Program(subtreeMutationAtRoot(parent.root(), rng));
        };
    }

    private Node subtreeMutation(Node root, Rng rng) {
        int index = rng.nextInt(root.size());
        Node target = root.at(index);
        int room = Math.max(0, Math.min(4, maxDepth - root.levelOf(index)));
        return root.replace(index, generate(target.type(), rng.nextInt(room + 1), false, rng));
    }

    /** Fallback that always changes the program and respects the limits. */
    private Node subtreeMutationAtRoot(Node root, Rng rng) {
        for (int attempt = 0; attempt < 50; attempt++) {
            Node n = generate(Op.Type.ACTION, 1 + rng.nextInt(Math.max(1, Math.min(3, maxDepth))), false, rng);
            if (withinLimits(n) && !n.equals(root)) {
                return n;
            }
        }
        return root.op() == Op.STOP ? Node.of(Op.FORWARD) : Node.of(Op.STOP);
    }

    static Node pointMutation(Node root, Rng rng) {
        int index = rng.nextInt(root.size());
        Node target = root.at(index);
        if (target.op() == Op.CONST) {
            return root.replace(index, Node.constant(target.value() + rng.gaussian() * 0.5));
        }
        List<Op> alternatives = target.op().sameSignature();
        if (alternatives.isEmpty()) {
            return null;
        }
        Op op = alternatives.get(rng.nextInt(alternatives.size()));
        return root.replace(index, Node.of(op, target.children()));
    }

    static Node constantMutation(Node root, double sigma, Rng rng) {
        List<Integer> constants = new ArrayList<>();
        for (int i = 0; i < root.size(); i++) {
            if (root.at(i).op() == Op.CONST) {
                constants.add(i);
            }
        }
        if (constants.isEmpty()) {
            return null;
        }
        Node out = root;
        for (int i : constants) {
            out = out.replace(i, Node.constant(out.at(i).value() + rng.gaussian() * sigma));
        }
        return out;
    }

    @Override
    public CrossoverOperator<Program> crossover(Params params) {
        SolutionSpace.operator(params, "crossover", CROSSOVERS, representation());
        return (a, b, rng) -> {
            // Crossover points in A are restricted to types that also occur in B (the action root always does).
            List<Integer> shared = new ArrayList<>();
            for (Op.Type t : Op.Type.values()) {
                if (!b.root().indicesOf(t).isEmpty()) {
                    shared.addAll(a.root().indicesOf(t));
                }
            }
            int ia = pick(a.root(), shared, rng);
            Op.Type type = a.root().at(ia).type();
            List<Integer> compatible = b.root().indicesOf(type);
            int ib = pick(b.root(), compatible, rng);
            Node sa = a.root().at(ia);
            Node sb = b.root().at(ib);
            Node c1 = a.root().replace(ia, sb);
            Node c2 = b.root().replace(ib, sa);
            return List.of(withinLimits(c1) ? new Program(c1) : a, withinLimits(c2) ? new Program(c2) : b);
        };
    }

    /** Koza's 90/10 rule: internal nodes with probability 0.9 when there are any. */
    private static int pick(Node root, List<Integer> allowed, Rng rng) {
        List<Integer> pool = allowed == null ? range(root.size()) : allowed;
        List<Integer> internal = new ArrayList<>();
        for (int i : pool) {
            if (!root.at(i).children().isEmpty()) {
                internal.add(i);
            }
        }
        List<Integer> from = !internal.isEmpty() && rng.chance(0.9) ? internal : pool;
        return from.get(rng.nextInt(from.size()));
    }

    private static List<Integer> range(int n) {
        List<Integer> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            out.add(i);
        }
        return out;
    }

    @Override
    public Neighbourhood<Program> neighbourhood(Params params) {
        return new Neighbourhood<>() {
            @Override
            public List<Program> all(Program p) {
                List<Program> out = new ArrayList<>();
                Node root = p.root();
                for (int i = 0; i < root.size(); i++) {
                    Node n = root.at(i);
                    if (n.op() == Op.CONST) {
                        for (double step : new double[] {-1, -0.25, 0.25, 1}) {
                            out.add(new Program(root.replace(i, Node.constant(n.value() + step))));
                        }
                    } else {
                        for (Op op : n.op().sameSignature()) {
                            out.add(new Program(root.replace(i, Node.of(op, n.children()))));
                        }
                    }
                }
                return out;
            }

            @Override
            public Program random(Program p, Rng rng) {
                for (int attempt = 0; attempt < 20; attempt++) {
                    Node n = pointMutation(p.root(), rng);
                    if (n != null && !n.equals(p.root())) {
                        return new Program(n);
                    }
                }
                return new Program(subtreeMutationAtRoot(p.root(), rng));
            }
        };
    }

    @Override
    public double distance(Program a, Program b) {
        List<String> x = a.tokens();
        List<String> y = b.tokens();
        int n = Math.min(x.size(), 64);
        int m = Math.min(y.size(), 64);
        int[] prev = new int[m + 1];
        int[] cur = new int[m + 1];
        for (int j = 0; j <= m; j++) {
            prev[j] = j;
        }
        for (int i = 1; i <= n; i++) {
            cur[0] = i;
            for (int j = 1; j <= m; j++) {
                int cost = x.get(i - 1).equals(y.get(j - 1)) ? 0 : 1;
                cur[j] = Math.min(Math.min(cur[j - 1] + 1, prev[j] + 1), prev[j - 1] + cost);
            }
            int[] t = prev;
            prev = cur;
            cur = t;
        }
        return (double) prev[m] / Math.max(1, Math.max(n, m));
    }

    @Override
    public List<String> mutationOperators() {
        return MUTATIONS;
    }

    @Override
    public List<String> crossoverOperators() {
        return CROSSOVERS;
    }

    @Override
    public int dimension() {
        return maxSize;
    }
}
