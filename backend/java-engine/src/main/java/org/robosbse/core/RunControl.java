package org.robosbse.core;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Thread-safe pause / resume / cancel switch shared between a running search and its controller
 * (the protocol server or a test). The search thread calls {@link #checkpoint()} before each
 * fitness evaluation.
 */
public final class RunControl {

    private final ReentrantLock lock = new ReentrantLock();
    private final Condition resumed = lock.newCondition();
    private boolean paused;
    private boolean cancelled;
    private long pausedNanos;

    public void pause() {
        lock.lock();
        try {
            paused = true;
        } finally {
            lock.unlock();
        }
    }

    public void resume() {
        lock.lock();
        try {
            paused = false;
            resumed.signalAll();
        } finally {
            lock.unlock();
        }
    }

    public void cancel() {
        lock.lock();
        try {
            cancelled = true;
            resumed.signalAll();
        } finally {
            lock.unlock();
        }
    }

    public boolean isPaused() {
        lock.lock();
        try {
            return paused;
        } finally {
            lock.unlock();
        }
    }

    public boolean isCancelled() {
        lock.lock();
        try {
            return cancelled;
        } finally {
            lock.unlock();
        }
    }

    /** Total time spent blocked in {@link #checkpoint()} while paused. */
    public long pausedNanos() {
        lock.lock();
        try {
            return pausedNanos;
        } finally {
            lock.unlock();
        }
    }

    /** Blocks while paused; throws {@link SearchTermination} if the run was cancelled. */
    public void checkpoint() {
        lock.lock();
        try {
            if (paused && !cancelled) {
                long start = System.nanoTime();
                while (paused && !cancelled) {
                    resumed.awaitUninterruptibly();
                }
                pausedNanos += System.nanoTime() - start;
            }
            if (cancelled) {
                throw new SearchTermination(StopReason.CANCELLED);
            }
        } finally {
            lock.unlock();
        }
    }
}
