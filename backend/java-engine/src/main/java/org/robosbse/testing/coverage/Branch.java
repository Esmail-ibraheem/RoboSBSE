package org.robosbse.testing.coverage;

/**
 * One instrumented decision (an {@code if}, loop condition or conditional expression) of a controller.
 * Each decision has two outcomes, {@code true} and {@code false}; "branch coverage" counts covered
 * outcomes.
 *
 * <p>{@code parent}/{@code parentOutcome} name the decision outcome this decision is control-dependent
 * on (null for decisions that are evaluated in every control step). They define the approach level:
 * a decision that was never evaluated is approached through its chain of parents.
 *
 * @param id            stable identifier, e.g. {@code goal-reached}
 * @param description   what the condition tests, in source terms
 * @param parent        id of the controlling decision, or null
 * @param parentOutcome outcome of the parent under which this decision is evaluated
 */
public record Branch(String id, String description, String parent, boolean parentOutcome) {

    public static Branch root(String id, String description) {
        return new Branch(id, description, null, false);
    }

    public static Branch under(String id, String description, String parent, boolean parentOutcome) {
        return new Branch(id, description, parent, parentOutcome);
    }

    /** Label of one outcome, e.g. {@code goal-reached:true}. */
    public static String outcome(String id, boolean outcome) {
        return id + ":" + outcome;
    }
}
