package org.robosbse.geneticprogramming;

import java.util.ArrayList;
import java.util.List;

/**
 * The primitives of the robot-decision language. Every primitive has a return type and typed
 * arguments, so programs are well-typed by construction (strongly typed GP, Montana 1995):
 *
 * <pre>
 *   ACTION  forward | turnLeft | turnRight | veerLeft | veerRight | stop | if(BOOL, ACTION, ACTION)
 *   BOOL    obstacleAhead | goalLeft | goalAhead | NUM &lt; NUM | NUM &gt; NUM | and | or | not
 *   NUM     front | left | right | goalBearing | goalDistance | constant | NUM + NUM | NUM - NUM | NUM * NUM
 * </pre>
 *
 * Sensor terminals read the minimum range in a sector (front: |angle| &lt;= 30 degrees, left: 30 to 120,
 * right: -120 to -30), in metres. {@code goalBearing} is the goal direction relative to the heading
 * (radians, positive = left). The language has no loops, so a program's cost per control tick is
 * bounded by its size.
 */
public enum Op {
    FORWARD("forward", Type.ACTION),
    TURN_LEFT("turnLeft", Type.ACTION),
    TURN_RIGHT("turnRight", Type.ACTION),
    VEER_LEFT("veerLeft", Type.ACTION),
    VEER_RIGHT("veerRight", Type.ACTION),
    STOP("stop", Type.ACTION),
    IF("if", Type.ACTION, Type.BOOL, Type.ACTION, Type.ACTION),

    OBSTACLE_AHEAD("obstacleAhead", Type.BOOL),
    GOAL_LEFT("goalLeft", Type.BOOL),
    GOAL_AHEAD("goalAhead", Type.BOOL),
    LT("<", Type.BOOL, Type.NUM, Type.NUM),
    GT(">", Type.BOOL, Type.NUM, Type.NUM),
    AND("and", Type.BOOL, Type.BOOL, Type.BOOL),
    OR("or", Type.BOOL, Type.BOOL, Type.BOOL),
    NOT("not", Type.BOOL, Type.BOOL),

    FRONT("front", Type.NUM),
    LEFT("left", Type.NUM),
    RIGHT("right", Type.NUM),
    GOAL_BEARING("goalBearing", Type.NUM),
    GOAL_DISTANCE("goalDistance", Type.NUM),
    CONST("const", Type.NUM),
    ADD("+", Type.NUM, Type.NUM, Type.NUM),
    SUB("-", Type.NUM, Type.NUM, Type.NUM),
    MUL("*", Type.NUM, Type.NUM, Type.NUM);

    /** Value types of the language. */
    public enum Type {
        ACTION, BOOL, NUM
    }

    private final String symbol;
    private final Type type;
    private final List<Type> args;

    Op(String symbol, Type type, Type... args) {
        this.symbol = symbol;
        this.type = type;
        this.args = List.of(args);
    }

    public String symbol() {
        return symbol;
    }

    public Type type() {
        return type;
    }

    public List<Type> args() {
        return args;
    }

    public int arity() {
        return args.size();
    }

    public boolean terminal() {
        return args.isEmpty();
    }

    /** Primitives with the given return type, terminals or functions. */
    public static List<Op> of(Type type, boolean terminal) {
        List<Op> out = new ArrayList<>();
        for (Op op : values()) {
            if (op.type == type && op.terminal() == terminal) {
                out.add(op);
            }
        }
        return out;
    }

    /** Primitives with exactly the same signature (return and argument types), for point mutation. */
    public List<Op> sameSignature() {
        List<Op> out = new ArrayList<>();
        for (Op op : values()) {
            if (op != this && op.type == type && op.args.equals(args) && (op == CONST) == (this == CONST)) {
                out.add(op);
            }
        }
        return out;
    }

    public static Op bySymbol(String symbol) {
        for (Op op : values()) {
            if (op.symbol.equals(symbol)) {
                return op;
            }
        }
        return null;
    }
}
