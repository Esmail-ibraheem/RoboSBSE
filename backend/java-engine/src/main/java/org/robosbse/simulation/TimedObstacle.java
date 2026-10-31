package org.robosbse.simulation;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import org.robosbse.geometry.Shape;

/**
 * An obstacle that exists only during a time window: a dynamic environmental event such as a door that
 * closes or a pallet that is removed.
 *
 * @param shape the obstacle while present
 * @param from  appearance time (s)
 * @param until disappearance time (s); null means it stays
 * @param label optional name
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record TimedObstacle(Shape shape, double from, Double until, String label) {

    public TimedObstacle {
        if (shape == null) {
            throw new IllegalArgumentException("a timed obstacle needs a shape");
        }
    }

    public boolean active(double t) {
        return t >= from && (until == null || t < until);
    }
}
