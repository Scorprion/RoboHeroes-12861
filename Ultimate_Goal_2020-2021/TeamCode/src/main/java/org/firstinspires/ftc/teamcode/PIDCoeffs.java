package org.firstinspires.ftc.teamcode;

public class PIDCoeffs {
    private double p, i, d;
    public PIDCoeffs(double P, double I, double D) {
        this.p = P;
        this.i = I;
        this.d = D;
    }

    public double getP() {
        return this.p;
    }

    public double getI() {
        return this.i;
    }

    public double getD() {
        return this.d;
    }
}
