package org.robosbse.testing;

import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonInclude;

import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;

/**
 * Self-contained, reproducible description of a discovered failure ({@code robosbse.failure/1}).
 * It embeds everything needed to replay the episode — arena, scenario, controller and its
 * parameters, robot, testing settings including the noise seed — so a saved record works as a
 * regression test without the experiment that found it.
 *
 * @param violated   properties violated by the controller under test
 * @param reference  verdict of the correct controller on the same scenario (differential check)
 * @param provenance how the failure was found (experiment, algorithm, seed, evaluation index)
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonInclude(JsonInclude.Include.NON_NULL)
public record FailureRecord(String schema, String controller, Map<String, Object> controllerParams,
                            List<String> violated, List<Verdict> verdicts, String outcome, double time,
                            Scenario scenario, WorldMap arena, Map<String, Object> testing, RobotSpec robot,
                            double[] genome, Map<String, Object> reference, Map<String, Object> provenance,
                            Map<String, Object> minimization) {

    public static final String SCHEMA = "robosbse.failure/1";
}
