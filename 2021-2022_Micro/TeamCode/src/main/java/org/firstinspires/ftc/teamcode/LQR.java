package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class LQR {
    RealMatrix A;
    RealMatrix Q;
    RealMatrix Qf;
    RealMatrix R;

    public LQR(RealMatrix A, RealMatrix Q, RealMatrix Qf, RealMatrix R) {
        this.A = A;
        this.Q = Q;
        this.Qf = Qf;
        this.R = R;
    }

    public RealVector calculate(RealMatrix B, RealVector error, int timesteps) {
        RealMatrix P = Qf;

        for(int i = timesteps; i > 0; i--) {
            SingularValueDecomposition svd = new SingularValueDecomposition(R.add(B.transpose().multiply(P).multiply(B)));
            DecompositionSolver solver = svd.getSolver();
            RealMatrix pinv = solver.getInverse();

            P = Q.add(A.transpose().multiply(P).multiply(A).subtract(A.transpose().multiply(P).multiply(B).multiply(pinv).multiply(B.transpose()).multiply(P).multiply(A)));
        }

        SingularValueDecomposition svd = new SingularValueDecomposition(R.add(B.transpose().multiply(P).multiply(B)));
        DecompositionSolver solver = svd.getSolver();
        RealMatrix pinv = solver.getInverse();
        RealMatrix K = pinv.multiply(B.transpose()).multiply(P).multiply(A);
        return K.preMultiply(error);
    }
}
