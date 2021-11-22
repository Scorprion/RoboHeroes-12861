package org.firstinspires.ftc.teamcode;

public class PIDCoeffs {
    public double p, i, d, f;
    public PIDCoeffs(double P, double I, double D, double F) {
        this.p = P;
        this.i = I;
        this.d = D;
        this.f = F;
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

    public double getF() { return this.f; }

    public void setP(double P) { this.p = P; }

    public void setI(double I) { this.i = I; }

    public void setD(double D) { this.d = D; }

    public void setF(double F) { this.f = F; }

}
