package org.robosbse;

import org.robosbse.core.Evaluation;
import org.robosbse.core.Problem;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.core.representation.RealVectorSpace;

/**
 * Test problem with a known optimum: f(x) = sum (x_i - c_i)^2 on [-5, 5]^n, minimum 0 at
 * c = (1, 1.5, 2, ...). An optional target threshold allows stop-on-target tests.
 */
public final class Sphere implements Problem<RealVector> {

    private final RealVectorSpace space;
    private final double[] centre;
    private final double target;

    public Sphere(int dimension) {
        this(dimension, Double.NEGATIVE_INFINITY);
    }

    public Sphere(int dimension, double target) {
        this.space = RealVectorSpace.uniform(dimension, -5, 5);
        this.centre = new double[dimension];
        for (int i = 0; i < dimension; i++) {
            centre[i] = 1.0 + 0.5 * i;
        }
        this.target = target;
    }

    @Override
    public String type() {
        return "sphere";
    }

    @Override
    public SolutionSpace<RealVector> space() {
        return space;
    }

    @Override
    public Evaluation evaluate(RealVector x) {
        double sum = 0;
        for (int i = 0; i < x.size(); i++) {
            double d = x.get(i) - centre[i];
            sum += d * d;
        }
        return Evaluation.of(sum);
    }

    @Override
    public boolean isTarget(Evaluation evaluation) {
        return evaluation.fitness() <= target;
    }
}
