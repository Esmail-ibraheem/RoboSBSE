package org.robosbse.simulation;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import org.robosbse.geometry.Shape;

/**
 * A region with special rules.
 *
 * <ul>
 *   <li>{@code restricted}: the robot may drive through but should not; the time its body overlaps the
 *       zone is measured ({@code restrictedTime}) and penalised by route optimisation.</li>
 *   <li>{@code terrain}: rough ground; the robot's achievable speed is multiplied by {@code speedFactor}
 *       while its centre is inside.</li>
 * </ul>
 *
 * @param kind        {@code restricted} or {@code terrain}
 * @param shape       the region
 * @param speedFactor terrain speed multiplier in (0, 1] (ignored for restricted zones)
 * @param label       optional name
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record Zone(String kind, Shape shape, double speedFactor, String label) {

    public Zone {
        if (shape == null) {
            throw new IllegalArgumentException("a zone needs a shape");
        }
        kind = kind == null ? "restricted" : kind;
        if (!kind.equals("restricted") && !kind.equals("terrain")) {
            throw new IllegalArgumentException("zone kind must be 'restricted' or 'terrain'");
        }
        if (kind.equals("terrain") && !(speedFactor > 0 && speedFactor <= 1)) {
            throw new IllegalArgumentException("terrain speedFactor must be in (0, 1]");
        }
    }

    public boolean restricted() {
        return kind.equals("restricted");
    }
}
