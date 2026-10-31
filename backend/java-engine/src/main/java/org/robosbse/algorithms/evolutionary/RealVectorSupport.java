package org.robosbse.algorithms.evolutionary;

import org.robosbse.core.SearchContext;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

/** Helpers for algorithms that are only defined on continuous, box-constrained vectors. */
final class RealVectorSupport {

    private RealVectorSupport() {
    }

    /**
     * Narrows a context to real vectors, failing with a clear message for other representations
     * (e.g. PSO is not applied to permutations without a defined continuous encoding).
     */
    @SuppressWarnings("unchecked")
    static SearchContext<RealVector> realContext(SearchContext<?> context, String algorithm) {
        if (!(context.space() instanceof RealVectorSpace)) {
            throw new IllegalArgumentException(algorithm + " requires the real-vector representation, but the problem uses '"
                    + context.space().representation() + "'");
        }
        return (SearchContext<RealVector>) context;
    }
}
