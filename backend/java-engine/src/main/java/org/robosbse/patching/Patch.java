package org.robosbse.patching;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * A candidate patch: a set of edits, at most one per mutation point, kept sorted by point index so
 * that equal patches are equal values. The empty patch is the unchanged subject.
 */
public final class Patch {

    /** Replace the code at {@code point} by its alternative number {@code choice}. */
    public record Edit(int point, int choice) {
    }

    private static final Patch EMPTY = new Patch(List.of());

    private final List<Edit> edits;

    private Patch(List<Edit> sorted) {
        this.edits = List.copyOf(sorted);
    }

    public static Patch empty() {
        return EMPTY;
    }

    public static Patch of(List<Edit> edits) {
        List<Edit> sorted = new ArrayList<>(edits);
        sorted.sort(Comparator.comparingInt(Edit::point));
        for (int i = 1; i < sorted.size(); i++) {
            if (sorted.get(i).point() == sorted.get(i - 1).point()) {
                throw new IllegalArgumentException("two edits at point " + sorted.get(i).point());
            }
        }
        return new Patch(sorted);
    }

    public List<Edit> edits() {
        return edits;
    }

    public int size() {
        return edits.size();
    }

    public boolean isEmpty() {
        return edits.isEmpty();
    }

    public Edit at(int point) {
        for (Edit e : edits) {
            if (e.point() == point) {
                return e;
            }
        }
        return null;
    }

    public Patch with(Edit edit) {
        List<Edit> next = new ArrayList<>();
        for (Edit e : edits) {
            if (e.point() != edit.point()) {
                next.add(e);
            }
        }
        next.add(edit);
        return of(next);
    }

    public Patch without(int point) {
        List<Edit> next = new ArrayList<>();
        for (Edit e : edits) {
            if (e.point() != point) {
                next.add(e);
            }
        }
        return of(next);
    }

    /** Compact text form, e.g. {@code 12:3,40:0}; parsed by {@link #parse}. */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (Edit e : edits) {
            if (!sb.isEmpty()) {
                sb.append(',');
            }
            sb.append(e.point()).append(':').append(e.choice());
        }
        return sb.toString();
    }

    public static Patch parse(String text) {
        List<Edit> edits = new ArrayList<>();
        if (text != null && !text.isBlank()) {
            for (String part : text.split(",")) {
                String[] pc = part.trim().split(":");
                if (pc.length != 2) {
                    throw new IllegalArgumentException("patch edits look like <point>:<choice>, got '" + part + "'");
                }
                edits.add(new Edit(Integer.parseInt(pc[0].trim()), Integer.parseInt(pc[1].trim())));
            }
        }
        return of(edits);
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Patch p && p.edits.equals(edits);
    }

    @Override
    public int hashCode() {
        return edits.hashCode();
    }
}
