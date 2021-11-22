package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.Array2DRowFieldMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class KalmanFilter {

    RealVector X0;  // Previous State Vector (n x 1)
    RealVector X1;  // Current State Vector t+1 (ensures that we call predict beforehand)

    RealMatrix P0;  // Previous Covariance Matrix (n x n)
    RealMatrix P1;  // Current Covariance Matrix


    RealMatrix F;  // Transition Matrix
    RealMatrix B;  // Actuation Matrix (transition for our inputs i.e. motor voltages)
    RealMatrix H;  // Observation Matrix (how sensors measure from the previous state)

    RealMatrix Q;  // Environmental Uncertainty
    RealMatrix R;  // Sensor Uncertainty

    RealMatrix K;  // Kalman gain calculated at each step


    public KalmanFilter(RealVector init_X, RealMatrix init_cov) {
        this.X0 = init_X;  // X = MatrixUtils.createRealVector();
        this.X1 = null;

        this.P0 = init_cov;  // Q = Array2DRowFieldMatrix
        this.P1 = null;

        this.K = null;

    }

    /**
     * Predict the next state given the current state, inputs, and transition matrices
     *
     * @param A State Transition Matrix (n x n)
     * @param B Actuation Matrix (n x p)
     * @param U Input Vector (p x 1)
     * @param Q Environmental Uncertainty (n x n)
     */
    public void predict(RealMatrix A, RealMatrix B, RealVector U, RealMatrix Q) {
        // TODO: It may be post (just multiply), double check (transition matrix may be a param too)
        // State prediction equation: x1 = A x0 + B u
        this.X1 = A.preMultiply(this.X0).add(B.preMultiply(U));

        // Covariance prediction equation
        // P1 = A P0 A.T + Q
        this.P1 = A.preMultiply(this.P0).preMultiply(A.transpose()).add(Q);
    }


    /**
     * Updates the current state estimate from the previous prediction and given observations
     *
     * @param z current measurements/observations from system of interest
     */
    public void update(RealVector z, RealMatrix H, RealMatrix R) {
        if (this.X1 == null || this.P1 == null) {
            throw new RuntimeException("Call to predict before calling to update");
        }

        // Kalman gain
        this.K = this.P1.preMultiply(H.transpose())
                .preMultiply(MatrixUtils.inverse(H.preMultiply(this.P1).preMultiply(H.transpose()).add(R)));

        // TODO: Ensure this is ran after the prediction matrix (or create separate data structure for these values)
        this.X0 = this.X1.add(this.K.preMultiply(z.subtract(H.preMultiply(this.X1))));
        this.P0 = this.P1.subtract(this.K.preMultiply(H).preMultiply(this.P1));

        this.X1 = null;
        this.P1 = null;
    }

    public RealVector get_state() {
        return this.X0;
    }

    public RealMatrix get_cov() {
        return this.P0;
    }
}
