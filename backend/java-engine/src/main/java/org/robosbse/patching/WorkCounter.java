package org.robosbse.patching;

/**
 * Implemented by subject controllers that count their own computational work (e.g. obstacle-point
 * checks), so that genetic improvement can measure computational cost deterministically instead of
 * by noisy wall-clock timing.
 */
public interface WorkCounter {

    /** Units of work done since the last {@code reset}. */
    long work();
}
