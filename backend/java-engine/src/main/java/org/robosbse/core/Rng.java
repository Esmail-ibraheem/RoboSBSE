package org.robosbse.core;

import java.util.List;
import java.util.SplittableRandom;

/**
 * Deterministic pseudo-random number generator used by every stochastic component.
 *
 * <p>All randomness in the engine flows through explicitly seeded {@code Rng} instances, never
 * through {@link Math#random()} or unseeded generators, so that any experiment can be replayed
 * bit-for-bit from its configuration and seed. {@link #split()} derives statistically independent
 * child streams (e.g. one per island or per parallel run) without consuming the parent's sequence
 * in an order-dependent way.
 */
public final class Rng {

    private final SplittableRandom random;

    public Rng(long seed) {
        this.random = new SplittableRandom(seed);
    }

    private Rng(SplittableRandom random) {
        this.random = random;
    }

    /** Uniform double in [0, 1). */
    public double nextDouble() {
        return random.nextDouble();
    }

    /** Uniform double in [lo, hi). */
    public double uniform(double lo, double hi) {
        return lo + (hi - lo) * random.nextDouble();
    }

    /** Uniform int in [0, bound). */
    public int nextInt(int bound) {
        return random.nextInt(bound);
    }

    public long nextLong() {
        return random.nextLong();
    }

    /** Returns true with probability {@code p}. */
    public boolean chance(double p) {
        return random.nextDouble() < p;
    }

    /** Standard normal deviate. */
    public double gaussian() {
        return random.nextGaussian();
    }

    public <T> T pick(List<T> items) {
        return items.get(nextInt(items.size()));
    }

    /** In-place Fisher-Yates shuffle. */
    public <T> void shuffle(List<T> items) {
        for (int i = items.size() - 1; i > 0; i--) {
            int j = nextInt(i + 1);
            T tmp = items.get(i);
            items.set(i, items.get(j));
            items.set(j, tmp);
        }
    }

    /** Independent child stream. */
    public Rng split() {
        return new Rng(random.split());
    }
}
