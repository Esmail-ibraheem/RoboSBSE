package org.robosbse.testing.coverage;

import java.util.List;

/**
 * A controller whose decisions carry coverage probes. Coverage is only ever reported for controllers
 * that implement this interface; for all others it is "unavailable", never estimated.
 */
public interface Instrumented {

    /** The instrumented decisions, in slot order. */
    List<Branch> branches();

    /** Attaches the trace that records the next episode (call after {@code reset}). */
    void attach(CoverageTrace trace);

    /** One sentence on what was instrumented and how, for reports. */
    String instrumentation();
}
