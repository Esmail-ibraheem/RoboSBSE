package org.robosbse.testing.coverage;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Union of the outcomes covered by many episodes (all scenarios a search generated). For every outcome
 * it keeps how many scenarios covered it and one covering scenario genome for replay: the
 * lexicographically smallest, so the result does not depend on the order in which parallel
 * evaluations finish. Thread-safe.
 */
public final class CoverageAccumulator {

    private final List<Branch> branches;
    private final long[] trueCount;
    private final long[] falseCount;
    private final double[][] trueWitness;
    private final double[][] falseWitness;
    private long episodes;

    public CoverageAccumulator(List<Branch> branches) {
        this.branches = List.copyOf(branches);
        int n = branches.size();
        this.trueCount = new long[n];
        this.falseCount = new long[n];
        this.trueWitness = new double[n][];
        this.falseWitness = new double[n][];
    }

    public List<Branch> branches() {
        return branches;
    }

    public synchronized void add(CoverageTrace trace, double[] genome) {
        episodes++;
        for (int i = 0; i < branches.size(); i++) {
            if (trace.covered(i, true)) {
                trueCount[i]++;
                trueWitness[i] = smaller(trueWitness[i], genome);
            }
            if (trace.covered(i, false)) {
                falseCount[i]++;
                falseWitness[i] = smaller(falseWitness[i], genome);
            }
        }
    }

    /** Adds everything {@code other} recorded (same decision list). */
    public void merge(CoverageAccumulator other) {
        long[] t;
        long[] f;
        double[][] tw;
        double[][] fw;
        long e;
        synchronized (other) {
            t = other.trueCount.clone();
            f = other.falseCount.clone();
            tw = other.trueWitness.clone();
            fw = other.falseWitness.clone();
            e = other.episodes;
        }
        synchronized (this) {
            if (t.length != trueCount.length) {
                throw new IllegalArgumentException("cannot merge coverage of different decision lists");
            }
            episodes += e;
            for (int i = 0; i < t.length; i++) {
                trueCount[i] += t[i];
                falseCount[i] += f[i];
                trueWitness[i] = smaller(trueWitness[i], tw[i]);
                falseWitness[i] = smaller(falseWitness[i], fw[i]);
            }
        }
    }

    private static double[] smaller(double[] current, double[] candidate) {
        if (candidate == null) {
            return current;
        }
        if (current == null) {
            return candidate.clone();
        }
        for (int i = 0; i < Math.min(current.length, candidate.length); i++) {
            if (candidate[i] != current[i]) {
                return candidate[i] < current[i] ? candidate.clone() : current;
            }
        }
        return current;
    }

    public synchronized boolean covered(String id, boolean outcome) {
        int i = slot(id);
        return (outcome ? trueCount[i] : falseCount[i]) > 0;
    }

    public synchronized double[] witness(String id, boolean outcome) {
        int i = slot(id);
        double[] w = outcome ? trueWitness[i] : falseWitness[i];
        return w == null ? null : w.clone();
    }

    public synchronized int coveredOutcomes() {
        int n = 0;
        for (int i = 0; i < branches.size(); i++) {
            n += (trueCount[i] > 0 ? 1 : 0) + (falseCount[i] > 0 ? 1 : 0);
        }
        return n;
    }

    public int totalOutcomes() {
        return 2 * branches.size();
    }

    public synchronized long episodes() {
        return episodes;
    }

    private int slot(String id) {
        for (int i = 0; i < branches.size(); i++) {
            if (branches.get(i).id().equals(id)) {
                return i;
            }
        }
        throw new IllegalArgumentException("unknown decision '" + id + "'");
    }

    /** JSON-ready summary: per decision, scenario counts and witnesses for both outcomes. */
    public synchronized Map<String, Object> report() {
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("episodes", episodes);
        out.put("outcomes", totalOutcomes());
        out.put("covered", coveredOutcomes());
        out.put("coverage", totalOutcomes() == 0 ? 0.0 : (double) coveredOutcomes() / totalOutcomes());
        List<Map<String, Object>> list = new ArrayList<>();
        for (int i = 0; i < branches.size(); i++) {
            Branch b = branches.get(i);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", b.id());
            m.put("description", b.description());
            m.put("parent", b.parent() == null ? null : Branch.outcome(b.parent(), b.parentOutcome()));
            m.put("trueScenarios", trueCount[i]);
            m.put("falseScenarios", falseCount[i]);
            list.add(m);
        }
        out.put("branches", list);
        return out;
    }
}
