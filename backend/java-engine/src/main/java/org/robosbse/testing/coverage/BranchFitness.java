package org.robosbse.testing.coverage;

import java.util.List;

/**
 * Fitness for covering one decision outcome (minimised, 0 = covered), combining the approach level and
 * the normalised branch distance (Wegener et al.; McMinn):
 * <pre>
 *   fitness = approachLevel + d / (d + 1)
 * </pre>
 * If the target decision was evaluated, the approach level is 0 and {@code d} is the smallest branch
 * distance to the target outcome over the episode. Otherwise the search walks up the declared
 * control dependencies: the approach level counts the decisions between the target and the closest
 * ancestor that was evaluated, and {@code d} is that ancestor's distance to the outcome leading towards
 * the target.
 */
public final class BranchFitness {

    private BranchFitness() {
    }

    /** Approach level and branch distance of an outcome in a trace. */
    public record Value(int approachLevel, double distance, double fitness) {

        public boolean covered() {
            return fitness == 0;
        }
    }

    public static Value of(CoverageTrace trace, String id, boolean outcome) {
        List<Branch> branches = trace.branches();
        int slot = trace.slot(id);
        boolean wanted = outcome;
        int level = 0;
        while (true) {
            if (trace.reached(slot)) {
                double d = trace.distance(slot, wanted);
                return new Value(level, d, level + (d == 0 ? 0 : d / (d + 1)));
            }
            Branch b = branches.get(slot);
            if (b.parent() == null) {
                // A root decision that never ran (e.g. the episode ended before the first step).
                return new Value(level + 1, Double.POSITIVE_INFINITY, level + 1);
            }
            wanted = b.parentOutcome();
            slot = trace.slot(b.parent());
            level++;
        }
    }
}
