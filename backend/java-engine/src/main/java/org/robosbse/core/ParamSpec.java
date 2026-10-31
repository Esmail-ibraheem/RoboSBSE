package org.robosbse.core;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Declaration of one configurable parameter. Specs drive validation and let the UI render only the
 * parameters that apply to the selected algorithm or problem.
 */
public record ParamSpec(String name, Type type, Object defaultValue, Double min, Double max,
                        List<String> options, String description) {

    public enum Type { INT, DOUBLE, BOOL, CHOICE, STRING }

    public ParamSpec {
        options = options == null ? List.of() : List.copyOf(options);
    }

    public static ParamSpec integer(String name, int def, int min, int max, String description) {
        return new ParamSpec(name, Type.INT, def, (double) min, (double) max, null, description);
    }

    public static ParamSpec real(String name, double def, double min, double max, String description) {
        return new ParamSpec(name, Type.DOUBLE, def, min, max, null, description);
    }

    public static ParamSpec bool(String name, boolean def, String description) {
        return new ParamSpec(name, Type.BOOL, def, null, null, null, description);
    }

    public static ParamSpec choice(String name, List<String> options, String description) {
        return new ParamSpec(name, Type.CHOICE, options.get(0), null, null, options, description);
    }

    public static ParamSpec text(String name, String def, String description) {
        return new ParamSpec(name, Type.STRING, def, null, null, null, description);
    }

    /** Returns a list of human-readable problems with {@code params}; empty when valid. */
    public static List<String> validate(List<ParamSpec> specs, Params params) {
        List<String> errors = new ArrayList<>();
        Set<String> known = new HashSet<>();
        for (ParamSpec spec : specs) {
            known.add(spec.name());
            if (!params.has(spec.name())) {
                continue;
            }
            try {
                spec.check(params);
            } catch (IllegalArgumentException e) {
                errors.add(e.getMessage());
            }
        }
        for (String key : params.asMap().keySet()) {
            if (!known.contains(key)) {
                errors.add("unknown parameter '" + key + "' (known: " + known + ")");
            }
        }
        return errors;
    }

    private void check(Params params) {
        switch (type) {
            case INT -> {
                int v = params.getInt(name, 0);
                checkRange(v);
            }
            case DOUBLE -> checkRange(params.getDouble(name, 0));
            case BOOL -> {
                Object raw = params.raw(name);
                if (!(raw instanceof Boolean) && !"true".equals(raw) && !"false".equals(raw)) {
                    throw new IllegalArgumentException("parameter '" + name + "' must be a boolean");
                }
            }
            case CHOICE -> {
                String v = params.getString(name, "");
                if (!options.contains(v)) {
                    throw new IllegalArgumentException(
                            "parameter '" + name + "' must be one of " + options + " but was '" + v + "'");
                }
            }
            case STRING -> {
                // free text
            }
        }
    }

    private void checkRange(double v) {
        if (Double.isNaN(v) || (min != null && v < min) || (max != null && v > max)) {
            throw new IllegalArgumentException(
                    "parameter '" + name + "'=" + v + " outside [" + min + ", " + max + "]");
        }
    }
}
