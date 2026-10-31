package org.robosbse.patching;

import java.util.List;

/**
 * A location in a subject's source where the restricted patch operators may change the code.
 *
 * @param index        position in the catalogue's deterministic discovery order
 * @param kind         which operator applies
 * @param method       enclosing method or field name
 * @param line         1-based source line
 * @param column       1-based source column
 * @param original     the code at this point (operator symbol, literal or condition)
 * @param alternatives the allowed replacements, in a fixed order
 * @param conflicts    indices of points that cannot be edited together with this one (a condition and
 *                     the points nested inside it)
 */
public record MutationPoint(int index, Kind kind, String method, int line, int column, String original,
                            List<String> alternatives, List<Integer> conflicts) {

    /** The restricted operator set of the specification. */
    public enum Kind {
        /** Replace a comparison operator ({@code < <= > >= == !=}). */
        COMPARISON,
        /** Replace an arithmetic operator ({@code + - * /}). */
        ARITHMETIC,
        /** Replace a logical operator ({@code && ||}). */
        LOGICAL,
        /** Change a numeric constant (0, 1, x0.01, x0.1, x0.5, x2, x10, x100, sign; integers 0, 1, +-1, /2, x2, sign). */
        LITERAL,
        /**
         * Replace the condition of an if, loop or conditional expression by {@code true} (never for loops),
         * {@code false}, its negation, or another condition of the same method.
         */
        CONDITION
    }

    /** One-line description, e.g. {@code L42 safeSpeed: '<' -> '<='}. */
    public String describe(int alternative) {
        return "L" + line + " " + method + ": " + kind.name().toLowerCase() + " '" + original + "' -> '"
                + alternatives.get(alternative) + "'";
    }
}
