package org.robosbse.core;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Delta debugging (Zeller and Hildebrandt, 2002): reduces a failure-inducing list (obstacles of a test
 * scenario, edits of a patch) to a 1-minimal sublist that still satisfies the test.
 */
public final class DeltaDebugging {

    private DeltaDebugging() {
    }

    /**
     * Delta debugging: a 1-minimal sublist of {@code items} for which {@code test} still holds
     * ({@code test(items)} is assumed true).
     */
    public static <T> List<T> ddmin(List<T> items, Predicate<List<T>> test) {
        List<T> current = new ArrayList<>(items);
        int n = 2;
        while (current.size() >= 2) {
            List<List<T>> chunks = split(current, n);
            boolean reduced = false;
            for (List<T> chunk : chunks) {
                if (test.test(chunk)) {
                    current = new ArrayList<>(chunk);
                    n = 2;
                    reduced = true;
                    break;
                }
            }
            if (!reduced && n > 2) {
                for (List<T> chunk : chunks) {
                    List<T> complement = new ArrayList<>(current);
                    complement.removeAll(chunk);
                    if (test.test(complement)) {
                        current = complement;
                        n = Math.max(n - 1, 2);
                        reduced = true;
                        break;
                    }
                }
            }
            if (!reduced) {
                if (n >= current.size()) {
                    break;
                }
                n = Math.min(current.size(), 2 * n);
            }
        }
        if (current.size() == 1 && test.test(List.of())) {
            current = new ArrayList<>();
        }
        return current;
    }

    private static <T> List<List<T>> split(List<T> items, int n) {
        List<List<T>> chunks = new ArrayList<>();
        int start = 0;
        for (int i = 0; i < n; i++) {
            int end = start + (items.size() - start) / (n - i);
            if (end > start) {
                chunks.add(new ArrayList<>(items.subList(start, end)));
            }
            start = end;
        }
        return chunks;
    }

}
