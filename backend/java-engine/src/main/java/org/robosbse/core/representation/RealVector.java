package org.robosbse.core.representation;

import java.util.Arrays;

/** Immutable vector of real-valued decision variables. */
public final class RealVector {

    private final double[] values;

    public RealVector(double[] values) {
        this.values = values.clone();
    }

    /** Wraps without copying; only for operators that own the freshly created array. */
    static RealVector wrap(double[] values) {
        return new RealVector(values, true);
    }

    private RealVector(double[] values, boolean ignored) {
        this.values = values;
    }

    public double get(int i) {
        return values[i];
    }

    public int size() {
        return values.length;
    }

    public double[] toArray() {
        return values.clone();
    }

    public RealVector with(int i, double value) {
        double[] copy = values.clone();
        copy[i] = value;
        return wrap(copy);
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof RealVector other && Arrays.equals(values, other.values);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(values);
    }

    @Override
    public String toString() {
        return Arrays.toString(values);
    }
}
