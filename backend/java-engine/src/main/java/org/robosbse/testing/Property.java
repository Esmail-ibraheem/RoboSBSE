package org.robosbse.testing;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * Declared properties of the robot-controller software, checked by the test oracle. Each property
 * has a robustness value in [-1, 1]: negative exactly when it is violated, and — where the
 * measurement allows — smaller the closer the episode came to a violation, which gives the search a
 * gradient. Properties without a meaningful distance measure are scored +1 when satisfied.
 */
public enum Property {

    COLLISION("collision", true, "The robot body never penetrates an obstacle."),
    BOUNDS("bounds", true, "The robot stays inside the environment."),
    VALID_COMMAND("valid-command", true, "The controller returns a finite command and does not crash."),
    VELOCITY("velocity", true, "Commanded velocities stay within the robot's configured limits."),
    COMPLETION("completion", true, "The controller never reports completion while outside the goal tolerance."),
    CLEARANCE("clearance", false, "The robot keeps at least the configured minimum clearance to obstacles."),
    LIVENESS("liveness", false,
            "If the goal is reachable and the battery suffices, the robot reaches it within the time limit.");

    private final String id;
    private final boolean safety;
    private final String description;

    Property(String id, boolean safety, String description) {
        this.id = id;
        this.safety = safety;
        this.description = description;
    }

    public String id() {
        return id;
    }

    /** Part of the default "safety" property set. */
    public boolean safety() {
        return safety;
    }

    public String description() {
        return description;
    }

    public static Property byId(String id) {
        for (Property p : values()) {
            if (p.id.equals(id)) {
                return p;
            }
        }
        throw new IllegalArgumentException("unknown property '" + id + "'");
    }

    /** Parses {@code safety}, {@code all}, or a comma-separated list of property ids. */
    public static Set<Property> parseSet(String spec) {
        if (spec == null || spec.isBlank() || spec.equals("safety")) {
            Set<Property> set = EnumSet.noneOf(Property.class);
            for (Property p : values()) {
                if (p.safety) {
                    set.add(p);
                }
            }
            return set;
        }
        if (spec.equals("all")) {
            return EnumSet.allOf(Property.class);
        }
        Set<Property> set = EnumSet.noneOf(Property.class);
        for (String part : spec.split(",")) {
            set.add(byId(part.trim()));
        }
        return set;
    }

    public static List<String> ids() {
        List<String> ids = new ArrayList<>();
        for (Property p : values()) {
            ids.add(p.id);
        }
        return ids;
    }
}
