package org.robosbse.experiments;

import java.util.Arrays;

/**
 * Descriptive and inferential statistics for comparing randomised algorithms, following the common
 * SBSE guidelines (Arcuri and Briand): non-parametric Mann-Whitney U test for significance and the
 * Vargha-Delaney A12 effect size.
 */
public final class Statistics {

    private Statistics() {
    }

    /** Five-number summary plus mean and sample standard deviation. */
    public record Summary(int n, double mean, double sd, double min, double q1, double median, double q3, double max) {
    }

    public static Summary summarize(double[] values) {
        int n = values.length;
        if (n == 0) {
            return new Summary(0, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }
        double[] s = values.clone();
        Arrays.sort(s);
        double mean = 0;
        for (double v : s) {
            mean += v;
        }
        mean /= n;
        double var = 0;
        for (double v : s) {
            var += (v - mean) * (v - mean);
        }
        double sd = n > 1 ? Math.sqrt(var / (n - 1)) : 0;
        return new Summary(n, mean, sd, s[0], quantile(s, 0.25), quantile(s, 0.5), quantile(s, 0.75), s[n - 1]);
    }

    /** Quantile of sorted data with linear interpolation (Hyndman-Fan type 7, as in R and NumPy). */
    public static double quantile(double[] sorted, double p) {
        if (sorted.length == 1) {
            return sorted[0];
        }
        double h = (sorted.length - 1) * p;
        int lo = (int) Math.floor(h);
        int hi = Math.min(sorted.length - 1, lo + 1);
        return sorted[lo] + (h - lo) * (sorted[hi] - sorted[lo]);
    }

    /**
     * Two-sided Mann-Whitney U test (normal approximation with tie and continuity correction).
     *
     * @param u      U statistic of the first sample
     * @param z      standardised statistic
     * @param pValue two-sided p-value
     */
    public record MannWhitney(double u, double z, double pValue) {
    }

    public static MannWhitney mannWhitney(double[] a, double[] b) {
        int n1 = a.length;
        int n2 = b.length;
        if (n1 == 0 || n2 == 0) {
            return new MannWhitney(Double.NaN, Double.NaN, Double.NaN);
        }
        int n = n1 + n2;
        double[][] all = new double[n][2];
        for (int i = 0; i < n1; i++) {
            all[i] = new double[] {a[i], 0};
        }
        for (int i = 0; i < n2; i++) {
            all[n1 + i] = new double[] {b[i], 1};
        }
        Arrays.sort(all, (x, y) -> Double.compare(x[0], y[0]));
        double rankSumA = 0;
        double tieTerm = 0;
        int i = 0;
        while (i < n) {
            int j = i;
            while (j + 1 < n && all[j + 1][0] == all[i][0]) {
                j++;
            }
            double avgRank = (i + j) / 2.0 + 1;
            int t = j - i + 1;
            tieTerm += (double) t * t * t - t;
            for (int k = i; k <= j; k++) {
                if (all[k][1] == 0) {
                    rankSumA += avgRank;
                }
            }
            i = j + 1;
        }
        double u = rankSumA - n1 * (n1 + 1) / 2.0;
        double mu = n1 * (double) n2 / 2.0;
        double sigma2 = n1 * (double) n2 / 12.0 * ((n + 1) - tieTerm / (n * (n - 1.0)));
        if (!(sigma2 > 0)) {
            return new MannWhitney(u, 0, 1.0);
        }
        double diff = u - mu;
        double z = (diff - 0.5 * Math.signum(diff)) / Math.sqrt(sigma2);
        double p = Math.min(1.0, 2 * (1 - normalCdf(Math.abs(z))));
        return new MannWhitney(u, z, p);
    }

    /**
     * Vargha-Delaney A12 for minimisation: probability that a run of A yields a <em>lower</em>
     * (better) value than a run of B, counting ties as one half. 0.5 = no difference; values above
     * 0.56 / 0.64 / 0.71 are conventionally small / medium / large effects in favour of A.
     */
    public static double a12(double[] a, double[] b) {
        if (a.length == 0 || b.length == 0) {
            return Double.NaN;
        }
        double wins = 0;
        for (double x : a) {
            for (double y : b) {
                if (x < y) {
                    wins += 1;
                } else if (x == y) {
                    wins += 0.5;
                }
            }
        }
        return wins / (a.length * (double) b.length);
    }

    /** Qualitative magnitude of an A12 value. */
    public static String a12Magnitude(double a12) {
        double d = Math.abs(a12 - 0.5);
        if (Double.isNaN(d)) {
            return "n/a";
        }
        if (d < 0.06) {
            return "negligible";
        }
        if (d < 0.14) {
            return "small";
        }
        if (d < 0.21) {
            return "medium";
        }
        return "large";
    }

    /** Holm-Bonferroni adjusted p-values (same order as the input). */
    public static double[] holm(double[] p) {
        int m = p.length;
        Integer[] order = new Integer[m];
        for (int i = 0; i < m; i++) {
            order[i] = i;
        }
        Arrays.sort(order, (x, y) -> Double.compare(p[x], p[y]));
        double[] adjusted = new double[m];
        double running = 0;
        for (int k = 0; k < m; k++) {
            int idx = order[k];
            double value = Math.min(1.0, (m - k) * p[idx]);
            running = Math.max(running, value);
            adjusted[idx] = running;
        }
        return adjusted;
    }

    /** Standard normal CDF via the complementary error function (Numerical Recipes erfc, |rel err| < 1.2e-7). */
    public static double normalCdf(double z) {
        return 0.5 * erfc(-z / Math.sqrt(2));
    }

    private static double erfc(double x) {
        double z = Math.abs(x);
        double t = 1 / (1 + 0.5 * z);
        double r = t * Math.exp(-z * z - 1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418
                + t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587
                + t * (-0.82215223 + t * 0.17087277)))))))));
        return x >= 0 ? r : 2 - r;
    }
}
