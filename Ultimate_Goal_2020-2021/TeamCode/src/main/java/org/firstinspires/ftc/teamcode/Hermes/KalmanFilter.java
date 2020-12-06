package org.firstinspires.ftc.teamcode.Hermes;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class KalmanFilter {
    RealMatrix Q;  // Sensor Noise
    RealMatrix P;  // Covariance Matrix
    RealMatrix F;  // Transition Matrix
    RealMatrix H;  // Observation Matrix

    RealVector X;  // State Matrix
    RealVector U;  // Inputs

    double R;  // Measurement noise


    public KalmanFilter(RealVector state, RealVector inputs, RealMatrix sensor_noise, RealMatrix covariance, RealMatrix transition, RealMatrix observation) {
        this.X = state;  // X = MatrixUtils.createRealVector();
        this.U = inputs;

        this.Q = sensor_noise;
        this.P = covariance;
        this.F = transition;
        this.H = observation;
    }
}
