package org.robosbse.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Immutable, loosely typed parameter map (decoded from JSON configuration).
 *
 * <p>Typed getters convert JSON numbers/strings/booleans leniently and fall back to the supplied
 * default when a key is absent. Validation against declared {@link ParamSpec}s happens once, when
 * an experiment is started, so that misspelled or out-of-range parameters fail fast.
 */
public final class Params {

    private static final Params EMPTY = new Params(Map.of());

    private final Map<String, Object> values;

    private Params(Map<String, ?> values) {
        this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
    }

    public static Params empty() {
        return EMPTY;
    }

    public static Params of(Map<String, ?> values) {
        return values == null || values.isEmpty() ? EMPTY : new Params(values);
    }

    public static Params of(Object... keyValues) {
        if (keyValues.length % 2 != 0) {
            throw new IllegalArgumentException("expected key/value pairs");
        }
        Map<String, Object> map = new LinkedHashMap<>();
        for (int i = 0; i < keyValues.length; i += 2) {
            map.put((String) keyValues[i], keyValues[i + 1]);
        }
        return new Params(map);
    }

    public boolean has(String key) {
        return values.containsKey(key) && values.get(key) != null;
    }

    public Object raw(String key) {
        return values.get(key);
    }

    public double getDouble(String key, double fallback) {
        Object v = values.get(key);
        if (v == null) {
            return fallback;
        }
        if (v instanceof Number n) {
            return n.doubleValue();
        }
        try {
            return Double.parseDouble(v.toString());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("parameter '" + key + "' is not a number: " + v);
        }
    }

    public int getInt(String key, int fallback) {
        Object v = values.get(key);
        if (v == null) {
            return fallback;
        }
        double d = getDouble(key, fallback);
        if (d != Math.rint(d)) {
            throw new IllegalArgumentException("parameter '" + key + "' must be an integer: " + v);
        }
        return (int) d;
    }

    public long getLong(String key, long fallback) {
        Object v = values.get(key);
        if (v == null) {
            return fallback;
        }
        if (v instanceof Number n) {
            return n.longValue();
        }
        return (long) getDouble(key, fallback);
    }

    public boolean getBool(String key, boolean fallback) {
        Object v = values.get(key);
        if (v == null) {
            return fallback;
        }
        if (v instanceof Boolean b) {
            return b;
        }
        return Boolean.parseBoolean(v.toString());
    }

    public String getString(String key, String fallback) {
        Object v = values.get(key);
        return v == null ? fallback : v.toString();
    }

    public Params with(String key, Object value) {
        Map<String, Object> copy = new LinkedHashMap<>(values);
        copy.put(key, value);
        return new Params(copy);
    }

    /** Values from {@code overrides} take precedence over this map. */
    public Params merge(Params overrides) {
        Map<String, Object> copy = new LinkedHashMap<>(values);
        copy.putAll(overrides.values);
        return new Params(copy);
    }

    public Map<String, Object> asMap() {
        return values;
    }

    @Override
    public String toString() {
        return values.toString();
    }
}
