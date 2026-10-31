package org.robosbse.core.representation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;

import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.Move;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SolutionSpace;

/**
 * Permutations of {@code n} elements — the representation of ordering problems such as warehouse
 * task scheduling (the permutation is a priority list that a decoder turns into a feasible schedule).
 *
 * <p>Operators (selected by name; {@code "default"} = first):
 * <ul>
 *   <li>mutation: {@code swap} (exchange two elements), {@code insert} (move one element),
 *       {@code inversion} (reverse a segment, 2-opt), {@code scramble} (shuffle a segment);
 *       one move per mutation;</li>
 *   <li>crossover: {@code order} (OX, Davis), {@code pmx} (partially mapped), {@code cycle} (CX) —
 *       all produce valid permutations;</li>
 *   <li>neighbourhood: {@code swap} (all pairs, attribute = the two elements),
 *       {@code insert} (attribute = moved element), {@code inversion} (attribute = segment ends).</li>
 * </ul>
 */
public final class PermutationSpace implements SolutionSpace<Permutation> {

    public static final List<String> MUTATIONS = List.of("swap", "insert", "inversion", "scramble");
    public static final List<String> CROSSOVERS = List.of("order", "pmx", "cycle");
    public static final List<String> NEIGHBOURHOODS = List.of("swap", "insert", "inversion");

    private final int n;

    public PermutationSpace(int n) {
        if (n < 2) {
            throw new IllegalArgumentException("permutations need at least 2 elements");
        }
        this.n = n;
    }

    @Override
    public String representation() {
        return "permutation";
    }

    @Override
    public int dimension() {
        return n;
    }

    @Override
    public Permutation random(Rng rng) {
        int[] a = Permutation.identity(n).toArray();
        for (int i = n - 1; i > 0; i--) {
            int j = rng.nextInt(i + 1);
            int t = a[i];
            a[i] = a[j];
            a[j] = t;
        }
        return Permutation.wrap(a);
    }

    @Override
    public List<String> mutationOperators() {
        return MUTATIONS;
    }

    @Override
    public List<String> crossoverOperators() {
        return CROSSOVERS;
    }

    @Override
    public List<String> neighbourhoods() {
        return NEIGHBOURHOODS;
    }

    /** Two distinct positions i < j. */
    private int[] twoPositions(Rng rng) {
        int i = rng.nextInt(n);
        int j = rng.nextInt(n - 1);
        if (j >= i) {
            j++;
        }
        return new int[] {Math.min(i, j), Math.max(i, j)};
    }

    static int[] swap(int[] a, int i, int j) {
        int[] b = a.clone();
        int t = b[i];
        b[i] = b[j];
        b[j] = t;
        return b;
    }

    /** Removes the element at {@code from} and re-inserts it at {@code to}. */
    static int[] insert(int[] a, int from, int to) {
        int[] b = a.clone();
        int v = b[from];
        if (from < to) {
            System.arraycopy(a, from + 1, b, from, to - from);
        } else {
            System.arraycopy(a, to, b, to + 1, from - to);
        }
        b[to] = v;
        return b;
    }

    static int[] reverse(int[] a, int i, int j) {
        int[] b = a.clone();
        for (int lo = i, hi = j; lo < hi; lo++, hi--) {
            int t = b[lo];
            b[lo] = b[hi];
            b[hi] = t;
        }
        return b;
    }

    @Override
    public MutationOperator<Permutation> mutation(Params params) {
        String name = SolutionSpace.operator(params, "mutation", MUTATIONS, representation());
        return (parent, rng) -> {
            int[] a = parent.toArray();
            int[] p = twoPositions(rng);
            return Permutation.wrap(switch (name) {
                case "swap" -> swap(a, p[0], p[1]);
                case "insert" -> rng.chance(0.5) ? insert(a, p[0], p[1]) : insert(a, p[1], p[0]);
                case "inversion" -> reverse(a, p[0], p[1]);
                default -> {
                    int[] b = a.clone();
                    for (int k = p[1]; k > p[0]; k--) {
                        int r = p[0] + rng.nextInt(k - p[0] + 1);
                        int t = b[k];
                        b[k] = b[r];
                        b[r] = t;
                    }
                    yield Arrays.equals(a, b) ? swap(a, p[0], p[1]) : b;
                }
            });
        };
    }

    @Override
    public CrossoverOperator<Permutation> crossover(Params params) {
        String name = SolutionSpace.operator(params, "crossover", CROSSOVERS, representation());
        return (x, y, rng) -> {
            int[] a = x.toArray();
            int[] b = y.toArray();
            return switch (name) {
                case "order" -> {
                    int[] p = twoPositions(rng);
                    yield List.of(Permutation.wrap(orderCrossover(a, b, p[0], p[1])),
                            Permutation.wrap(orderCrossover(b, a, p[0], p[1])));
                }
                case "pmx" -> {
                    int[] p = twoPositions(rng);
                    yield List.of(Permutation.wrap(pmx(a, b, p[0], p[1])), Permutation.wrap(pmx(b, a, p[0], p[1])));
                }
                default -> List.of(Permutation.wrap(cycle(a, b)), Permutation.wrap(cycle(b, a)));
            };
        };
    }

    /** OX: keep {@code keep[i..j]}, fill the rest with the other parent's order starting after j. */
    static int[] orderCrossover(int[] keep, int[] other, int i, int j) {
        int len = keep.length;
        int[] child = new int[len];
        boolean[] used = new boolean[len];
        for (int k = i; k <= j; k++) {
            child[k] = keep[k];
            used[keep[k]] = true;
        }
        int pos = (j + 1) % len;
        for (int k = 0; k < len; k++) {
            int v = other[(j + 1 + k) % len];
            if (!used[v]) {
                child[pos] = v;
                used[v] = true;
                pos = (pos + 1) % len;
            }
        }
        return child;
    }

    /** PMX: copy {@code keep[i..j]}, place the other parent's genes following the segment mapping. */
    static int[] pmx(int[] keep, int[] other, int i, int j) {
        int len = keep.length;
        int[] child = new int[len];
        Arrays.fill(child, -1);
        boolean[] inSegment = new boolean[len];
        for (int k = i; k <= j; k++) {
            child[k] = keep[k];
            inSegment[keep[k]] = true;
        }
        for (int k = i; k <= j; k++) {
            int v = other[k];
            if (inSegment[v]) {
                continue;
            }
            int pos = k;
            while (pos >= i && pos <= j) {
                pos = indexOf(other, keep[pos]);
            }
            child[pos] = v;
        }
        for (int k = 0; k < len; k++) {
            if (child[k] == -1) {
                child[k] = other[k];
            }
        }
        return child;
    }

    /** CX: cycles alternate between the parents. */
    static int[] cycle(int[] a, int[] b) {
        int len = a.length;
        int[] child = new int[len];
        Arrays.fill(child, -1);
        int[] posInA = new int[len];
        for (int k = 0; k < len; k++) {
            posInA[a[k]] = k;
        }
        boolean fromA = true;
        for (int start = 0; start < len; start++) {
            if (child[start] != -1) {
                continue;
            }
            int pos = start;
            do {
                child[pos] = fromA ? a[pos] : b[pos];
                pos = posInA[b[pos]];
            } while (pos != start);
            fromA = !fromA;
        }
        return child;
    }

    private static int indexOf(int[] a, int v) {
        for (int k = 0; k < a.length; k++) {
            if (a[k] == v) {
                return k;
            }
        }
        return -1;
    }

    @Override
    public Neighbourhood<Permutation> neighbourhood(Params params) {
        String name = SolutionSpace.operator(params, "neighbourhood", NEIGHBOURHOODS, representation());
        return new Neighbourhood<>() {
            @Override
            public List<Permutation> all(Permutation s) {
                List<Permutation> out = new ArrayList<>();
                for (Move<Permutation> m : moves(s)) {
                    out.add(m.result());
                }
                return out;
            }

            @Override
            public List<Move<Permutation>> moves(Permutation s) {
                int[] a = s.toArray();
                List<Move<Permutation>> out = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) {
                        switch (name) {
                            case "swap" -> {
                                if (i < j) {
                                    String attr = "swap:" + Math.min(a[i], a[j]) + "," + Math.max(a[i], a[j]);
                                    out.add(new Move<>(Permutation.wrap(swap(a, i, j)), attr, attr));
                                }
                            }
                            case "insert" -> {
                                // Moving to j = i + 1 equals the adjacent swap covered by moving i+1 to i.
                                if (i != j && j != i + 1) {
                                    String attr = "move:" + a[i];
                                    out.add(new Move<>(Permutation.wrap(insert(a, i, j)), attr, attr));
                                }
                            }
                            default -> {
                                if (i < j) {
                                    String attr = "rev:" + i + "," + j;
                                    out.add(new Move<>(Permutation.wrap(reverse(a, i, j)), attr, attr));
                                }
                            }
                        }
                    }
                }
                return out;
            }

            @Override
            public Permutation random(Permutation s, Rng rng) {
                int[] a = s.toArray();
                int[] p = twoPositions(rng);
                return Permutation.wrap(switch (name) {
                    case "swap" -> swap(a, p[0], p[1]);
                    case "insert" -> rng.chance(0.5) ? insert(a, p[0], p[1]) : insert(a, p[1], p[0]);
                    default -> reverse(a, p[0], p[1]);
                });
            }
        };
    }

    /** Fraction of positions holding different elements (normalised Hamming distance). */
    @Override
    public double distance(Permutation a, Permutation b) {
        int diff = 0;
        for (int i = 0; i < n; i++) {
            if (a.get(i) != b.get(i)) {
                diff++;
            }
        }
        return (double) diff / n;
    }

    @Override
    public boolean enumerable() {
        return true;
    }

    /** All n! permutations in lexicographic order (Narayana's next-permutation algorithm). */
    @Override
    public Iterator<Permutation> enumerate() {
        return new Iterator<>() {
            private int[] next = Permutation.identity(n).toArray();

            @Override
            public boolean hasNext() {
                return next != null;
            }

            @Override
            public Permutation next() {
                if (next == null) {
                    throw new NoSuchElementException();
                }
                int[] current = next.clone();
                next = successor(next);
                return Permutation.wrap(current);
            }
        };
    }

    private static int[] successor(int[] a) {
        int[] b = a.clone();
        int i = b.length - 2;
        while (i >= 0 && b[i] >= b[i + 1]) {
            i--;
        }
        if (i < 0) {
            return null;
        }
        int j = b.length - 1;
        while (b[j] <= b[i]) {
            j--;
        }
        int t = b[i];
        b[i] = b[j];
        b[j] = t;
        for (int lo = i + 1, hi = b.length - 1; lo < hi; lo++, hi--) {
            t = b[lo];
            b[lo] = b[hi];
            b[hi] = t;
        }
        return b;
    }
}
