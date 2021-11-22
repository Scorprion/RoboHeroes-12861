package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.analysis.function.Exp;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class BayesOpt {

}

class GaussProcess {
    public void fit(RealMatrix X1, RealMatrix y) {

    }

    public void predict(RealMatrix X2) {

    }

    private RealVector kernel(RealMatrix X1, RealMatrix X2, double sigma, double gamma) {
        RealVector X1Norm = getL2Norm(X1);
        X1Norm.ebeMultiply(X1Norm);  // Inplace??

        RealVector X2Norm = getL2Norm(X2);
        X2Norm.ebeMultiply(X2Norm);

        RealVector eucDist = X1Norm.add(X2Norm).subtract((RealVector) X1.preMultiply(X2.transpose()));
        return eucDist.map(new Exp()).mapMultiply(Math.pow(sigma, 2));
    }

    private RealVector getL2Norm(RealMatrix x) {
        RealVector norm = null;
        for (int i = 0; i < x.getColumnDimension(); i++) {
            double eucDist = x.getRowVector(i).getNorm();
            norm.append(eucDist);
        }
        return norm;
    }

}
