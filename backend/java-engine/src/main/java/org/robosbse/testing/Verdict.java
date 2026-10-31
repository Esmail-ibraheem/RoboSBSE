package org.robosbse.testing;

/**
 * Oracle verdict for one property in one episode.
 *
 * @param property   property id
 * @param applicable false when the property does not apply (e.g. liveness for an unreachable goal)
 * @param violated   true when the property is violated
 * @param robustness normalised robustness in [-1, 1]; negative iff violated
 * @param detail     human-readable explanation
 */
public record Verdict(String property, boolean applicable, boolean violated, double robustness, String detail) {
}
