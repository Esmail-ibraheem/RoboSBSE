package org.robosbse.geneticprogramming;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Immutable AST node. Nodes are addressed by their preorder index within a tree (the root is 0);
 * {@link #at} and {@link #replace} navigate and rebuild trees by index.
 */
public final class Node {

    private final Op op;
    private final double value;
    private final List<Node> children;
    private final int size;
    private final int depth;

    private Node(Op op, double value, List<Node> children) {
        if (children.size() != op.arity()) {
            throw new IllegalArgumentException(op.symbol() + " needs " + op.arity() + " arguments, got " + children.size());
        }
        for (int i = 0; i < children.size(); i++) {
            if (children.get(i).op.type() != op.args().get(i)) {
                throw new IllegalArgumentException("argument " + (i + 1) + " of " + op.symbol() + " must be "
                        + op.args().get(i) + ", got " + children.get(i).op.type());
            }
        }
        this.op = op;
        this.value = value;
        this.children = List.copyOf(children);
        int s = 1;
        int d = 0;
        for (Node c : this.children) {
            s += c.size;
            d = Math.max(d, c.depth);
        }
        this.size = s;
        this.depth = this.children.isEmpty() ? 0 : d + 1;
    }

    public static Node of(Op op, Node... children) {
        if (op == Op.CONST) {
            throw new IllegalArgumentException("use Node.constant for constants");
        }
        return new Node(op, 0, List.of(children));
    }

    public static Node of(Op op, List<Node> children) {
        return new Node(op, 0, children);
    }

    /** Numeric constant, rounded to 0.01 so that printed programs parse back exactly. */
    public static Node constant(double value) {
        return new Node(Op.CONST, Math.round(value * 100) / 100.0, List.of());
    }

    public Op op() {
        return op;
    }

    public double value() {
        return value;
    }

    public List<Node> children() {
        return children;
    }

    public int size() {
        return size;
    }

    /** Depth of the subtree: 0 for a terminal. */
    public int depth() {
        return depth;
    }

    public Op.Type type() {
        return op.type();
    }

    /** The node with the given preorder index. */
    public Node at(int index) {
        if (index == 0) {
            return this;
        }
        int offset = 1;
        for (Node c : children) {
            if (index < offset + c.size) {
                return c.at(index - offset);
            }
            offset += c.size;
        }
        throw new IndexOutOfBoundsException(index);
    }

    /** Copy of this tree with the node at {@code index} replaced. */
    public Node replace(int index, Node replacement) {
        if (index == 0) {
            return replacement;
        }
        int offset = 1;
        List<Node> next = new ArrayList<>(children);
        for (int i = 0; i < children.size(); i++) {
            Node c = children.get(i);
            if (index < offset + c.size) {
                next.set(i, c.replace(index - offset, replacement));
                return new Node(op, value, next);
            }
            offset += c.size;
        }
        throw new IndexOutOfBoundsException(index);
    }

    /** Depth of the node at {@code index} below this node (the root has level 0). */
    public int levelOf(int index) {
        if (index == 0) {
            return 0;
        }
        int offset = 1;
        for (Node c : children) {
            if (index < offset + c.size) {
                return 1 + c.levelOf(index - offset);
            }
            offset += c.size;
        }
        throw new IndexOutOfBoundsException(index);
    }

    /** Preorder indices of the nodes with the given type. */
    public List<Integer> indicesOf(Op.Type type) {
        List<Integer> out = new ArrayList<>();
        collect(type, 0, out);
        return out;
    }

    private int collect(Op.Type type, int offset, List<Integer> out) {
        if (op.type() == type) {
            out.add(offset);
        }
        int next = offset + 1;
        for (Node c : children) {
            next = c.collect(type, next, out);
        }
        return next;
    }

    /** Preorder token list (symbols, constants formatted), used for distances. */
    public void tokens(List<String> out) {
        out.add(op == Op.CONST ? format(value) : op.symbol());
        for (Node c : children) {
            c.tokens(out);
        }
    }

    static String format(double v) {
        String s = String.format(Locale.ROOT, "%.2f", v);
        return s.equals("-0.00") ? "0.00" : s;
    }

    /** S-expression, e.g. {@code (if (< front 0.80) turnLeft forward)}. */
    @Override
    public String toString() {
        if (op == Op.CONST) {
            return format(value);
        }
        if (children.isEmpty()) {
            return op.symbol();
        }
        StringBuilder sb = new StringBuilder("(").append(op.symbol());
        for (Node c : children) {
            sb.append(' ').append(c);
        }
        return sb.append(')').toString();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Node n && n.op == op && Double.compare(n.value, value) == 0 && n.children.equals(children);
    }

    @Override
    public int hashCode() {
        return 31 * (31 * op.hashCode() + Double.hashCode(value)) + children.hashCode();
    }
}
