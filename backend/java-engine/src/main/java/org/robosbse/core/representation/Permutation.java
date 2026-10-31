package org.robosbse.core.representation;

import java.util.Arrays;

/** Immutable permutation of {@code 0 .. n-1} (an order or priority list). */
public final class Permutation {

    private final int[] order;

    public Permutation(int[] order) {
        this.order = order.clone();
        boolean[] seen = new boolean[order.length];
        for (int v : order) {
            if (v < 0 || v >= order.length || seen[v]) {
                throw new IllegalArgumentException("not a permutation: " + Arrays.toString(order));
            }
            seen[v] = true;
        }
    }

    /** Wraps without copying or checking; only for operators that own a freshly built valid array. */
    static Permutation wrap(int[] order) {
        return new Permutation(order, true);
    }

    private Permutation(int[] order, boolean ignored) {
        this.order = order;
    }

    public static Permutation identity(int n) {
        int[] a = new int[n];
        for (int i = 0; i < n; i++) {
            a[i] = i;
        }
        return wrap(a);
    }

    public int get(int position) {
        return order[position];
    }

    public int size() {
        return order.length;
    }

    public int[] toArray() {
        return order.clone();
    }

    /** Position of element {@code value}. */
    public int indexOf(int value) {
        for (int i = 0; i < order.length; i++) {
            if (order[i] == value) {
                return i;
            }
        }
        return -1;
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Permutation p && Arrays.equals(order, p.order);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(order);
    }

    @Override
    public String toString() {
        return Arrays.toString(order);
    }
}
