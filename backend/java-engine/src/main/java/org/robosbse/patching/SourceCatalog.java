package org.robosbse.patching;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.github.javaparser.JavaParser;
import com.github.javaparser.ParseResult;
import com.github.javaparser.ParserConfiguration;
import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.Node;
import com.github.javaparser.ast.body.BodyDeclaration;
import com.github.javaparser.ast.body.ConstructorDeclaration;
import com.github.javaparser.ast.body.FieldDeclaration;
import com.github.javaparser.ast.body.MethodDeclaration;
import com.github.javaparser.ast.body.TypeDeclaration;
import com.github.javaparser.ast.expr.BinaryExpr;
import com.github.javaparser.ast.expr.ConditionalExpr;
import com.github.javaparser.ast.expr.DoubleLiteralExpr;
import com.github.javaparser.ast.expr.Expression;
import com.github.javaparser.ast.expr.IntegerLiteralExpr;
import com.github.javaparser.ast.expr.StringLiteralExpr;
import com.github.javaparser.ast.expr.UnaryExpr;
import com.github.javaparser.ast.stmt.DoStmt;
import com.github.javaparser.ast.stmt.ForStmt;
import com.github.javaparser.ast.stmt.IfStmt;
import com.github.javaparser.ast.stmt.WhileStmt;
import com.github.javaparser.printer.lexicalpreservation.LexicalPreservingPrinter;

/**
 * The mutation points of a Java subject and the machinery to apply patches to it.
 *
 * <p>The source is parsed with JavaParser; mutation points are discovered in source order within the
 * members in scope (all members when the scope is empty). A patch is applied to a fresh parse of the
 * original, and the result is printed with the lexical-preserving printer, so a patched file differs
 * from the original only where it was edited. The same source and scope always give the same points.
 *
 * <p>Only the restricted operators of {@link MutationPoint.Kind} exist. They change operators, numeric
 * constants and conditions; they cannot add calls, statements or new identifiers, which bounds what a
 * candidate patch can do (it is still compiled and run in an isolated process with a timeout).
 */
public final class SourceCatalog {

    static final List<String> COMPARISONS = List.of("<", "<=", ">", ">=", "==", "!=");
    static final List<String> ARITHMETIC = List.of("+", "-", "*", "/");
    static final List<String> LOGICAL = List.of("&&", "||");

    private final String source;
    private final List<String> scope;
    private final List<MutationPoint> points;

    public SourceCatalog(String source, List<String> scope) {
        this.source = source;
        this.scope = List.copyOf(scope);
        this.points = build(discover(parse(source)));
    }

    public String source() {
        return source;
    }

    public List<MutationPoint> points() {
        return points;
    }

    /** Number of distinct single-edit patches. */
    public int singleEdits() {
        int n = 0;
        for (MutationPoint p : points) {
            n += p.alternatives().size();
        }
        return n;
    }

    static CompilationUnit parse(String text) {
        JavaParser parser = new JavaParser(new ParserConfiguration()
                .setLanguageLevel(ParserConfiguration.LanguageLevel.JAVA_21));
        ParseResult<CompilationUnit> r = parser.parse(text);
        if (!r.isSuccessful() || r.getResult().isEmpty()) {
            throw new IllegalArgumentException("cannot parse the subject: " + r.getProblems());
        }
        return r.getResult().get();
    }

    /** A discovered point: the AST node to change and what kind of change applies. */
    private record Found(Node node, MutationPoint.Kind kind, String member, boolean loop) {
    }

    private List<Found> discover(CompilationUnit cu) {
        List<Found> found = new ArrayList<>();
        for (TypeDeclaration<?> type : cu.getTypes()) {
            for (BodyDeclaration<?> member : type.getMembers()) {
                String name = memberName(member);
                if (name == null || (!scope.isEmpty() && !scope.contains(name))) {
                    continue;
                }
                Set<Node> skip = Collections.newSetFromMap(new IdentityHashMap<>());
                member.walk(Node.TreeTraversal.PREORDER, n -> visit(n, name, skip, found));
            }
        }
        return found;
    }

    private static String memberName(BodyDeclaration<?> member) {
        if (member instanceof MethodDeclaration m) {
            return m.getNameAsString();
        }
        if (member instanceof ConstructorDeclaration) {
            return "<init>";
        }
        if (member instanceof FieldDeclaration f) {
            return f.getVariable(0).getNameAsString();
        }
        return null;
    }

    private static void visit(Node n, String member, Set<Node> skip, List<Found> out) {
        if (skip.contains(n)) {
            return;
        }
        if (n instanceof IfStmt s) {
            out.add(new Found(s.getCondition(), MutationPoint.Kind.CONDITION, member, false));
        } else if (n instanceof WhileStmt s) {
            out.add(new Found(s.getCondition(), MutationPoint.Kind.CONDITION, member, true));
        } else if (n instanceof DoStmt s) {
            out.add(new Found(s.getCondition(), MutationPoint.Kind.CONDITION, member, true));
        } else if (n instanceof ForStmt s && s.getCompare().isPresent()) {
            out.add(new Found(s.getCompare().get(), MutationPoint.Kind.CONDITION, member, true));
        } else if (n instanceof ConditionalExpr c) {
            out.add(new Found(c.getCondition(), MutationPoint.Kind.CONDITION, member, false));
        } else if (n instanceof BinaryExpr b) {
            String op = b.getOperator().asString();
            if (COMPARISONS.contains(op)) {
                out.add(new Found(b, MutationPoint.Kind.COMPARISON, member, false));
            } else if (ARITHMETIC.contains(op) && !(b.getLeft() instanceof StringLiteralExpr)
                    && !(b.getRight() instanceof StringLiteralExpr)) {
                out.add(new Found(b, MutationPoint.Kind.ARITHMETIC, member, false));
            } else if (LOGICAL.contains(op)) {
                out.add(new Found(b, MutationPoint.Kind.LOGICAL, member, false));
            }
        } else if (n instanceof UnaryExpr u && u.getOperator() == UnaryExpr.Operator.MINUS
                && (u.getExpression() instanceof IntegerLiteralExpr || u.getExpression() instanceof DoubleLiteralExpr)) {
            skip.add(u.getExpression());
            out.add(new Found(u, MutationPoint.Kind.LITERAL, member, false));
        } else if (n instanceof IntegerLiteralExpr || n instanceof DoubleLiteralExpr) {
            out.add(new Found(n, MutationPoint.Kind.LITERAL, member, false));
        }
    }

    private static List<MutationPoint> build(List<Found> found) {
        Map<String, List<String>> conditionsByMember = new LinkedHashMap<>();
        for (Found f : found) {
            if (f.kind() == MutationPoint.Kind.CONDITION) {
                conditionsByMember.computeIfAbsent(f.member(), k -> new ArrayList<>()).add(f.node().toString());
            }
        }
        List<MutationPoint> out = new ArrayList<>();
        for (int i = 0; i < found.size(); i++) {
            Found f = found.get(i);
            String original;
            List<String> alternatives = new ArrayList<>();
            switch (f.kind()) {
                case COMPARISON, ARITHMETIC, LOGICAL -> {
                    original = ((BinaryExpr) f.node()).getOperator().asString();
                    List<String> family = f.kind() == MutationPoint.Kind.COMPARISON ? COMPARISONS
                            : f.kind() == MutationPoint.Kind.ARITHMETIC ? ARITHMETIC : LOGICAL;
                    for (String op : family) {
                        if (!op.equals(original)) {
                            alternatives.add(op);
                        }
                    }
                }
                case LITERAL -> {
                    original = f.node().toString();
                    boolean integer = f.node() instanceof IntegerLiteralExpr
                            || (f.node() instanceof UnaryExpr u && u.getExpression() instanceof IntegerLiteralExpr);
                    alternatives.addAll(literalAlternatives(literalValue(f.node()), integer));
                }
                default -> {
                    original = f.node().toString();
                    Set<String> alts = new LinkedHashSet<>();
                    if (!f.loop()) {
                        alts.add("true");
                    }
                    alts.add("false");
                    alts.add("!(" + original + ")");
                    for (String other : conditionsByMember.getOrDefault(f.member(), List.of())) {
                        alts.add(other);
                    }
                    alts.remove(original);
                    alternatives.addAll(alts);
                }
            }
            List<Integer> conflicts = new ArrayList<>();
            for (int j = 0; j < found.size(); j++) {
                if (j == i) {
                    continue;
                }
                Found g = found.get(j);
                boolean nested = (f.kind() == MutationPoint.Kind.CONDITION && f.node().isAncestorOf(g.node()))
                        || (g.kind() == MutationPoint.Kind.CONDITION && g.node().isAncestorOf(f.node()));
                if (nested) {
                    conflicts.add(j);
                }
            }
            int line = f.node().getBegin().map(p -> p.line).orElse(0);
            int column = f.node().getBegin().map(p -> p.column).orElse(0);
            out.add(new MutationPoint(i, f.kind(), f.member(), line, column, original, List.copyOf(alternatives),
                    List.copyOf(conflicts)));
        }
        return List.copyOf(out);
    }

    private static double literalValue(Node n) {
        if (n instanceof UnaryExpr u) {
            return -literalValue(u.getExpression());
        }
        if (n instanceof IntegerLiteralExpr i) {
            return i.asNumber().doubleValue();
        }
        return ((DoubleLiteralExpr) n).asDouble();
    }

    static List<String> literalAlternatives(double v, boolean integer) {
        Set<String> out = new LinkedHashSet<>();
        if (integer) {
            long x = Math.round(v);
            for (long c : new long[] {0, 1, x - 1, x + 1, x / 2, x * 2, -x}) {
                if (c != x) {
                    out.add(Long.toString(c));
                }
            }
        } else {
            double[] candidates = v == 0 ? new double[] {1, -1, 0.5}
                    : new double[] {0, 1, v * 0.01, v * 0.1, v * 0.5, v * 2, v * 10, v * 100, -v};
            String self = formatDouble(v);
            for (double c : candidates) {
                String s = formatDouble(c);
                if (!s.equals(self)) {
                    out.add(s);
                }
            }
        }
        return List.copyOf(out);
    }

    /** Six significant digits, always with a decimal point so that it stays a double literal. */
    static String formatDouble(double v) {
        if (v == 0) {
            return "0.0";
        }
        String s = new BigDecimal(v).round(new MathContext(6)).stripTrailingZeros().toPlainString();
        return s.contains(".") ? s : s + ".0";
    }

    /** Whether the patch's edits are pairwise compatible (no edit nested in an edited condition). */
    public boolean compatible(Patch patch) {
        for (Patch.Edit a : patch.edits()) {
            if (a.point() < 0 || a.point() >= points.size()
                    || a.choice() < 0 || a.choice() >= points.get(a.point()).alternatives().size()) {
                return false;
            }
            for (Patch.Edit b : patch.edits()) {
                if (points.get(a.point()).conflicts().contains(b.point())) {
                    return false;
                }
            }
        }
        return true;
    }

    /** The patched source. */
    public String apply(Patch patch) {
        if (patch.isEmpty()) {
            return source;
        }
        if (!compatible(patch)) {
            throw new IllegalArgumentException("incompatible or out-of-range edits: " + patch);
        }
        CompilationUnit cu = parse(source);
        LexicalPreservingPrinter.setup(cu);
        List<Found> found = discover(cu);
        for (Patch.Edit e : patch.edits()) {
            Found f = found.get(e.point());
            String alternative = points.get(e.point()).alternatives().get(e.choice());
            switch (f.kind()) {
                case COMPARISON, ARITHMETIC, LOGICAL -> ((BinaryExpr) f.node()).setOperator(operator(alternative));
                default -> {
                    Expression replacement = StaticJavaParser.parseExpression(alternative);
                    f.node().replace(replacement);
                }
            }
        }
        return LexicalPreservingPrinter.print(cu);
    }

    private static BinaryExpr.Operator operator(String symbol) {
        for (BinaryExpr.Operator op : BinaryExpr.Operator.values()) {
            if (op.asString().equals(symbol)) {
                return op;
            }
        }
        throw new IllegalArgumentException("unknown operator " + symbol);
    }

    /** Human-readable list of a patch's edits. */
    public List<String> describe(Patch patch) {
        List<String> out = new ArrayList<>();
        for (Patch.Edit e : patch.edits()) {
            out.add(points.get(e.point()).describe(e.choice()));
        }
        return out;
    }

    /** One row of a side-by-side diff; a null side means the line does not exist there. */
    public record DiffRow(Integer leftLine, String left, Integer rightLine, String right, boolean changed) {
    }

    /** Line-based side-by-side diff (longest common subsequence). */
    public static List<DiffRow> diff(String original, String patched) {
        String[] a = original.split("\\R", -1);
        String[] b = patched.split("\\R", -1);
        int n = a.length;
        int m = b.length;
        int[][] lcs = new int[n + 1][m + 1];
        for (int i = n - 1; i >= 0; i--) {
            for (int j = m - 1; j >= 0; j--) {
                lcs[i][j] = a[i].equals(b[j]) ? lcs[i + 1][j + 1] + 1 : Math.max(lcs[i + 1][j], lcs[i][j + 1]);
            }
        }
        List<DiffRow> rows = new ArrayList<>();
        int i = 0;
        int j = 0;
        while (i < n || j < m) {
            if (i < n && j < m && a[i].equals(b[j])) {
                rows.add(new DiffRow(i + 1, a[i], j + 1, b[j], false));
                i++;
                j++;
            } else if (i < n && j < m && lcs[i + 1][j] == lcs[i][j + 1]) {
                // A changed line: show old and new side by side.
                rows.add(new DiffRow(i + 1, a[i], j + 1, b[j], true));
                i++;
                j++;
            } else if (j >= m || (i < n && lcs[i + 1][j] >= lcs[i][j + 1])) {
                rows.add(new DiffRow(i + 1, a[i], null, null, true));
                i++;
            } else {
                rows.add(new DiffRow(null, null, j + 1, b[j], true));
                j++;
            }
        }
        return rows;
    }

    /** Only the changed rows of a diff, for compact reports. */
    public static List<DiffRow> changes(String original, String patched) {
        List<DiffRow> out = new ArrayList<>();
        for (DiffRow r : diff(original, patched)) {
            if (r.changed()) {
                out.add(r);
            }
        }
        return out;
    }

    /** Source text with whitespace runs collapsed, for comparing patches with a reference fix. */
    public static String normalise(String text) {
        return text.replaceAll("\\s+", " ").trim();
    }
}
