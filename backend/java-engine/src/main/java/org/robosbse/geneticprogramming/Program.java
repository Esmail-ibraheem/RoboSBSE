package org.robosbse.geneticprogramming;

import java.util.ArrayList;
import java.util.List;

/**
 * A controller program: a well-typed AST whose root returns an {@code ACTION}. Programs are immutable
 * values; equality is structural.
 *
 * <p>{@link #parse} reads the s-expression form printed by {@link #toString} and <b>rejects invalid
 * programs</b> (unknown symbols, wrong argument types or counts, a non-action root, size or depth above
 * the given limits) with an explanation instead of running them.
 */
public final class Program {

    private final Node root;

    public Program(Node root) {
        if (root.type() != Op.Type.ACTION) {
            throw new IllegalArgumentException("a program must return an action, got " + root.type());
        }
        this.root = root;
    }

    public Node root() {
        return root;
    }

    public int size() {
        return root.size();
    }

    public int depth() {
        return root.depth();
    }

    public static Program parse(String text, int maxSize, int maxDepth) {
        Parser p = new Parser(text);
        Node node = p.node();
        p.skipSpace();
        if (p.pos != text.length()) {
            throw new IllegalArgumentException("unexpected text after the program at position " + p.pos);
        }
        if (node.size() > maxSize) {
            throw new IllegalArgumentException("program has " + node.size() + " nodes, the limit is " + maxSize);
        }
        if (node.depth() > maxDepth) {
            throw new IllegalArgumentException("program has depth " + node.depth() + ", the limit is " + maxDepth);
        }
        return new Program(node);
    }

    /** Readable if/else form for reports and the UI. */
    public String pretty() {
        StringBuilder sb = new StringBuilder();
        pretty(root, 0, sb);
        return sb.toString();
    }

    private static void pretty(Node n, int indent, StringBuilder sb) {
        String pad = "  ".repeat(indent);
        if (n.op() == Op.IF) {
            sb.append(pad).append("if ").append(expression(n.children().get(0))).append(":\n");
            pretty(n.children().get(1), indent + 1, sb);
            sb.append(pad).append("else:\n");
            pretty(n.children().get(2), indent + 1, sb);
        } else {
            sb.append(pad).append(n.op().symbol()).append('\n');
        }
    }

    private static String expression(Node n) {
        return switch (n.op()) {
            case CONST -> Node.format(n.value());
            case LT, GT, ADD, SUB, MUL -> "(" + expression(n.children().get(0)) + " " + n.op().symbol() + " "
                    + expression(n.children().get(1)) + ")";
            case AND, OR -> "(" + expression(n.children().get(0)) + " " + n.op().symbol() + " "
                    + expression(n.children().get(1)) + ")";
            case NOT -> "not " + expression(n.children().get(0));
            default -> n.op().symbol();
        };
    }

    /** Preorder tokens, for distances. */
    public List<String> tokens() {
        List<String> out = new ArrayList<>();
        root.tokens(out);
        return out;
    }

    @Override
    public String toString() {
        return root.toString();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Program p && p.root.equals(root);
    }

    @Override
    public int hashCode() {
        return root.hashCode();
    }

    private static final class Parser {
        private final String text;
        private int pos;

        Parser(String text) {
            this.text = text;
        }

        void skipSpace() {
            while (pos < text.length() && Character.isWhitespace(text.charAt(pos))) {
                pos++;
            }
        }

        String token() {
            skipSpace();
            int start = pos;
            while (pos < text.length() && !Character.isWhitespace(text.charAt(pos)) && text.charAt(pos) != '('
                    && text.charAt(pos) != ')') {
                pos++;
            }
            if (start == pos) {
                throw new IllegalArgumentException("expected a symbol at position " + pos);
            }
            return text.substring(start, pos);
        }

        Node node() {
            skipSpace();
            if (pos >= text.length()) {
                throw new IllegalArgumentException("unexpected end of program");
            }
            if (text.charAt(pos) == '(') {
                pos++;
                String symbol = token();
                Op op = Op.bySymbol(symbol);
                if (op == null || op.terminal()) {
                    throw new IllegalArgumentException("unknown function '" + symbol + "'");
                }
                List<Node> children = new ArrayList<>();
                skipSpace();
                while (pos < text.length() && text.charAt(pos) != ')') {
                    children.add(node());
                    skipSpace();
                }
                if (pos >= text.length()) {
                    throw new IllegalArgumentException("missing ')' for " + symbol);
                }
                pos++;
                return Node.of(op, children);
            }
            String symbol = token();
            Op op = Op.bySymbol(symbol);
            if (op != null && op.terminal() && op != Op.CONST) {
                return Node.of(op);
            }
            try {
                double v = Double.parseDouble(symbol);
                if (!Double.isFinite(v)) {
                    throw new NumberFormatException();
                }
                return Node.constant(v);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("unknown symbol '" + symbol + "'");
            }
        }
    }
}
