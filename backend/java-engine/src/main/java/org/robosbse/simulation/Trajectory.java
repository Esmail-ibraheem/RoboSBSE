package org.robosbse.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Recorded episode in column-oriented form (compact JSON for replay in the UI). Frame {@code k}
 * consists of entry {@code k} of every column; {@code rays} holds one range array per frame, with
 * {@code null} entries for frames whose scan was lost.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public final class Trajectory {

    private final double dt;
    private double[] t = new double[64];
    private double[] x = new double[64];
    private double[] y = new double[64];
    private double[] theta = new double[64];
    private double[] v = new double[64];
    private double[] omega = new double[64];
    private double[] battery = new double[64];
    private double[] clearance = new double[64];
    private boolean[] scanValid = new boolean[64];
    private final List<float[]> rays;
    private int size;

    public Trajectory(double dt, boolean withRays) {
        this.dt = dt;
        this.rays = withRays ? new ArrayList<>() : null;
    }

    void add(double time, double px, double py, double heading, double speed, double turn, double charge,
             double minClearance, boolean valid, double[] ranges) {
        if (size == t.length) {
            int n = size * 2;
            t = Arrays.copyOf(t, n);
            x = Arrays.copyOf(x, n);
            y = Arrays.copyOf(y, n);
            theta = Arrays.copyOf(theta, n);
            v = Arrays.copyOf(v, n);
            omega = Arrays.copyOf(omega, n);
            battery = Arrays.copyOf(battery, n);
            clearance = Arrays.copyOf(clearance, n);
            scanValid = Arrays.copyOf(scanValid, n);
        }
        t[size] = time;
        x[size] = px;
        y[size] = py;
        theta[size] = heading;
        v[size] = speed;
        omega[size] = turn;
        battery[size] = charge;
        clearance[size] = minClearance;
        scanValid[size] = valid;
        if (rays != null) {
            if (ranges == null || !valid) {
                rays.add(null);
            } else {
                float[] f = new float[ranges.length];
                for (int i = 0; i < f.length; i++) {
                    f[i] = (float) ranges[i];
                }
                rays.add(f);
            }
        }
        size++;
    }

    /**
     * Adds a stationary frame (speed 0) at {@code time}: the start pose of a robot or the end of a
     * pause (loading, waiting), so that the replay holds the robot in place.
     */
    public void hold(double time, double px, double py, double heading) {
        add(time, px, py, heading, 0, 0, size > 0 ? battery[size - 1] : 1, size > 0 ? clearance[size - 1] : 0,
                false, null);
    }

    /**
     * Appends all frames of {@code other} shifted by {@code timeOffset} seconds (used to stitch
     * the legs of a multi-stop mission into one replay).
     */
    public void append(Trajectory other, double timeOffset) {
        for (int i = 0; i < other.size; i++) {
            double[] r = null;
            if (other.rays != null && other.rays.get(i) != null) {
                float[] f = other.rays.get(i);
                r = new double[f.length];
                for (int k = 0; k < f.length; k++) {
                    r[k] = f[k];
                }
            }
            add(other.t[i] + timeOffset, other.x[i], other.y[i], other.theta[i], other.v[i], other.omega[i],
                    other.battery[i], other.clearance[i], other.scanValid[i], r);
        }
    }

    /** Time of the last frame, or 0 when empty. */
    @JsonIgnore
    public double endTime() {
        return size == 0 ? 0 : t[size - 1];
    }

    @JsonProperty
    public double dt() {
        return dt;
    }

    @JsonIgnore
    public int size() {
        return size;
    }

    @JsonProperty("t")
    public double[] times() {
        return Arrays.copyOf(t, size);
    }

    @JsonProperty("x")
    public double[] xs() {
        return Arrays.copyOf(x, size);
    }

    @JsonProperty("y")
    public double[] ys() {
        return Arrays.copyOf(y, size);
    }

    @JsonProperty("theta")
    public double[] headings() {
        return Arrays.copyOf(theta, size);
    }

    @JsonProperty("v")
    public double[] speeds() {
        return Arrays.copyOf(v, size);
    }

    @JsonProperty("omega")
    public double[] turnRates() {
        return Arrays.copyOf(omega, size);
    }

    @JsonProperty("battery")
    public double[] batteries() {
        return Arrays.copyOf(battery, size);
    }

    @JsonProperty("clearance")
    public double[] clearances() {
        return Arrays.copyOf(clearance, size);
    }

    @JsonProperty("scanValid")
    public boolean[] scanValidity() {
        return Arrays.copyOf(scanValid, size);
    }

    @JsonProperty("rays")
    public List<float[]> rays() {
        return rays;
    }
}
