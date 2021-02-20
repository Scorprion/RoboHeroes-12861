package org.firstinspires.ftc.teamcode.Profiles;

public class TriangleProfile extends MotionProfile {
    double avgVeloc, totalTime, maxVeloc, maxAccel;

    public TriangleProfile(double distance, double totalTime) {
        this.avgVeloc = distance / totalTime;
        this.maxVeloc = this.avgVeloc * 2;
        this.maxAccel = this.maxVeloc / (totalTime / 2);
        this.totalTime = totalTime;
    }

    public TriangleProfile(double distance, double maxVeloc, double maxAccel) {

        double velocityBound = Math.sqrt(2 * distance * maxAccel);

        if (maxVeloc <= velocityBound) {
            this.maxVeloc = maxVeloc;

        } else {
            this.maxVeloc = velocityBound;
        }
        this.totalTime = 2 * this.maxVeloc / maxAccel;

    }

    @Override
    public double calculate(double time) {
        if(time < this.totalTime / 2) {
            return this.maxAccel * time; // time * Math.sqrt(this.accel * this.accel - 1);
        } else if (time < this.totalTime) {
            return this.maxVeloc - this.maxAccel * (time - this.totalTime / 2); // this.maxVeloc - time * Math.sqrt(this.accel * this.accel - 1);
        } else {
            return 0;
        }
    }
}
