package org.robosbse.experiments;

import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Candidate;
import org.robosbse.core.Problem;
import org.robosbse.core.SearchResult;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.WorldMap;

/**
 * A configured problem plus the problem-specific reporting that turns search results into
 * something a person can inspect: replays of solutions, reference baselines and discovered
 * failures.
 */
public interface ProblemInstance<S> extends AutoCloseable {

    Problem<S> problem();

    /** The environment the problem is defined on (inlined into results for reproducibility). */
    WorldMap map();

    /** Re-simulates a solution with recording enabled. */
    Map<String, Object> replay(S solution);

    /** Conventional reference results (e.g. A*), or null. */
    Map<String, Object> reference();

    /** Problem-specific description of target hits (e.g. failure records), or an empty list. */
    List<Object> describeTargets(SearchResult<S> result, Map<String, Object> provenance, boolean minimize, int limit);

    /** Rebuilds a solution from the {@code genome} array of its view (for replays requested by the UI). */
    S parse(JsonNode genome);

    /**
     * Makes the echoed problem configuration self-contained (inline the map, task sets, ...), so
     * that a result file alone reproduces the run.
     */
    default void normalise(ObjectNode problem) {
        problem.set("map", Json.tree(map()));
    }

    /**
     * Extra measurements of the final best solution that are not part of the search (e.g. its
     * performance on held-out validation maps). Not charged to the evaluation budget.
     */
    default Map<String, Object> bestExtras(S best) {
        return Map.of();
    }

    /**
     * Measurements the problem collected over the whole run (e.g. the branch coverage of every scenario
     * a test generator produced), keyed by result field name.
     */
    default Map<String, Object> runExtras() {
        return Map.of();
    }

    /** Whether the run reached the problem's notion of success (feasible route / failure found). */
    default boolean success(SearchResult<S> result) {
        return result.targetCount() > 0;
    }

    /** Result field (and output folder) for {@link #describeTargets}: failure records, patches, ... */
    default String targetsField() {
        return "failures";
    }

    /** Releases resources the problem holds (e.g. candidate worker processes). */
    @Override
    default void close() {
        if (problem() instanceof AutoCloseable c) {
            try {
                c.close();
            } catch (Exception e) {
                throw new IllegalStateException("could not close the problem: " + e.getMessage(), e);
            }
        }
    }

    /** Solution as a JSON-friendly object. */
    default Object view(Candidate<S> candidate) {
        return problem().view(candidate.solution());
    }
}
