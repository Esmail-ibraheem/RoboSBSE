package org.robosbse.patching;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.robosbse.core.CrossoverOperator;
import org.robosbse.core.MutationOperator;
import org.robosbse.core.Neighbourhood;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SolutionSpace;

/**
 * Representation of patches over a {@link SourceCatalog}, so that every generic algorithm can search
 * for patches (random patch search, GA, hill climbing, SA, tabu, ...).
 *
 * <ul>
 *   <li><b>random</b>: 1 edit (probability 0.7), 2 (0.2) or 3 (0.1) at random compatible points;</li>
 *   <li><b>mutation</b> {@code edit}: add a random edit (40%, or always for the empty patch), change the
 *       alternative of an existing edit (30%), or remove one (30%);</li>
 *   <li><b>crossover</b> {@code union}: each child takes every edit of either parent with probability 0.5,
 *       dropping edits that would conflict;</li>
 *   <li><b>neighbourhood</b>: every patch with one edit added, changed or removed (enumerable; large);
 *       random neighbour = the mutation;</li>
 *   <li><b>distance</b>: Jaccard distance between the edit sets.</li>
 * </ul>
 * Patches never exceed {@code maxEdits} edits and never combine conflicting edits.
 */
public final class PatchSpace implements SolutionSpace<Patch> {

    public static final List<String> MUTATIONS = List.of("edit");
    public static final List<String> CROSSOVERS = List.of("union");

    private final SourceCatalog catalog;
    private final int maxEdits;

    public PatchSpace(SourceCatalog catalog, int maxEdits) {
        if (catalog.points().isEmpty()) {
            throw new IllegalArgumentException("the subject has no mutation points in scope");
        }
        this.catalog = catalog;
        this.maxEdits = Math.max(1, maxEdits);
    }

    public SourceCatalog catalog() {
        return catalog;
    }

    @Override
    public String representation() {
        return "patch";
    }

    @Override
    public Patch random(Rng rng) {
        double u = rng.nextDouble();
        int k = Math.min(maxEdits, u < 0.7 ? 1 : u < 0.9 ? 2 : 3);
        Patch p = Patch.empty();
        for (int i = 0; i < k; i++) {
            Patch next = addRandom(p, rng);
            if (next != null) {
                p = next;
            }
        }
        return p.isEmpty() ? randomSingle(rng) : p;
    }

    private Patch randomSingle(Rng rng) {
        MutationPoint point = catalog.points().get(rng.nextInt(catalog.points().size()));
        return Patch.of(List.of(new Patch.Edit(point.index(), rng.nextInt(point.alternatives().size()))));
    }

    /** The patch plus one random compatible edit at a new point, or null if none fits. */
    private Patch addRandom(Patch p, Rng rng) {
        if (p.size() >= maxEdits) {
            return null;
        }
        for (int attempt = 0; attempt < 30; attempt++) {
            MutationPoint point = catalog.points().get(rng.nextInt(catalog.points().size()));
            if (p.at(point.index()) != null) {
                continue;
            }
            Patch next = p.with(new Patch.Edit(point.index(), rng.nextInt(point.alternatives().size())));
            if (catalog.compatible(next)) {
                return next;
            }
        }
        return null;
    }

    private Patch changeRandom(Patch p, Rng rng) {
        List<Patch.Edit> changeable = new ArrayList<>();
        for (Patch.Edit e : p.edits()) {
            if (catalog.points().get(e.point()).alternatives().size() > 1) {
                changeable.add(e);
            }
        }
        if (changeable.isEmpty()) {
            return null;
        }
        Patch.Edit e = changeable.get(rng.nextInt(changeable.size()));
        int n = catalog.points().get(e.point()).alternatives().size();
        int choice = (e.choice() + 1 + rng.nextInt(n - 1)) % n;
        return p.with(new Patch.Edit(e.point(), choice));
    }

    @Override
    public MutationOperator<Patch> mutation(Params params) {
        SolutionSpace.operator(params, "mutation", MUTATIONS, representation());
        return (parent, rng) -> {
            for (int attempt = 0; attempt < 20; attempt++) {
                double u = rng.nextDouble();
                Patch child;
                if (parent.isEmpty() || u < 0.4) {
                    child = addRandom(parent, rng);
                } else if (u < 0.7) {
                    child = changeRandom(parent, rng);
                } else {
                    child = parent.without(parent.edits().get(rng.nextInt(parent.size())).point());
                }
                if (child != null && !child.equals(parent) && (!child.isEmpty() || parent.size() > 1)) {
                    return child;
                }
            }
            return randomSingle(rng);
        };
    }

    @Override
    public CrossoverOperator<Patch> crossover(Params params) {
        SolutionSpace.operator(params, "crossover", CROSSOVERS, representation());
        return (a, b, rng) -> List.of(mix(a, b, rng), mix(b, a, rng));
    }

    private Patch mix(Patch first, Patch second, Rng rng) {
        Patch child = Patch.empty();
        List<Patch.Edit> pool = new ArrayList<>(first.edits());
        pool.addAll(second.edits());
        for (Patch.Edit e : pool) {
            if (child.size() < maxEdits && rng.chance(0.5) && child.at(e.point()) == null) {
                Patch next = child.with(e);
                if (catalog.compatible(next)) {
                    child = next;
                }
            }
        }
        return child.isEmpty() ? (first.isEmpty() ? randomSingle(rng) : first) : child;
    }

    @Override
    public Neighbourhood<Patch> neighbourhood(Params params) {
        MutationOperator<Patch> mutation = mutation(Params.empty());
        return new Neighbourhood<>() {
            @Override
            public List<Patch> all(Patch p) {
                List<Patch> out = new ArrayList<>();
                for (Patch.Edit e : p.edits()) {
                    out.add(p.without(e.point()));
                }
                for (MutationPoint point : catalog.points()) {
                    Patch.Edit current = p.at(point.index());
                    if (current == null && p.size() >= maxEdits) {
                        continue;
                    }
                    for (int c = 0; c < point.alternatives().size(); c++) {
                        if (current != null && current.choice() == c) {
                            continue;
                        }
                        Patch next = p.with(new Patch.Edit(point.index(), c));
                        if (catalog.compatible(next)) {
                            out.add(next);
                        }
                    }
                }
                out.removeIf(Patch::isEmpty);
                return out;
            }

            @Override
            public Patch random(Patch p, Rng rng) {
                return mutation.mutate(p, rng);
            }
        };
    }

    @Override
    public double distance(Patch a, Patch b) {
        Set<Patch.Edit> x = new HashSet<>(a.edits());
        Set<Patch.Edit> union = new HashSet<>(x);
        union.addAll(b.edits());
        if (union.isEmpty()) {
            return 0;
        }
        x.retainAll(new HashSet<>(b.edits()));
        return 1.0 - (double) x.size() / union.size();
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
    public int dimension() {
        return catalog.points().size();
    }
}
