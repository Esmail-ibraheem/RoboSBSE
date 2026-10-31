package org.robosbse.simulation.controllers;

import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;

/**
 * Robot controller software — the system under optimisation and under test.
 *
 * <p>A controller is reset once per episode and then queried every time step. It must not keep
 * hidden global state, so that episodes are independent and reproducible.
 */
public interface Controller {

    /** Identifier, e.g. {@code waypoint-follower} or {@code reactive-bug1-dropout}. */
    String id();

    /** Whether the controller reads the range sensor (sensing is skipped otherwise, for speed). */
    default boolean usesSensors() {
        return false;
    }

    void reset(ControllerSetup setup);

    Command step(Observation observation);
}
