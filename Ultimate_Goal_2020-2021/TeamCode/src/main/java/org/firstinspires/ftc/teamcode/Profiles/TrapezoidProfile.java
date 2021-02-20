package org.firstinspires.ftc.teamcode.Profiles;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

public class TrapezoidProfile extends MotionProfile {
    double[] times = new double[3];
    double acceleration, deceleration, velocity;

    public TrapezoidProfile(double distance, double maxVeloc, double accel, double decel) throws Exception {


        // Calculate time for accel and decel (accel * time = veloc)
        this.times[0] = maxVeloc / accel;
        this.times[2] = maxVeloc / decel;

        // Algebraic manipulation of the distance sum
        this.times[1] = (distance - maxVeloc) / maxVeloc;


        // Triangle or less than the triangle
        if (distance < 0.5 * accel * this.times[0] * this.times[0] || this.times[1] <= 0) {
            throw new Exception("The distance specified with the given velocity and acceleration constraints cannot be calculated. Consider using the TriangleProfile class.");
        }

        this.acceleration = accel;
        this.deceleration = decel;
        this.velocity = maxVeloc;
    }

    // Shortcut for when accel = decel
    public TrapezoidProfile(double distance, double maxVeloc, double maxAccel) {
        this.times[0] = maxVeloc / maxAccel;
        this.times[1] = (distance - maxVeloc) / distance;
        this.times[2] = maxVeloc / maxAccel;

        this.acceleration = maxAccel;
        this.deceleration = maxAccel;
        this.velocity = maxVeloc;
    }

    @Override
    public double calculate(double time) {
        if (time < this.times[0]) {
            return time * this.acceleration;
        } else if (time < this.times[1]) {
            return this.velocity;
        } else if (time < this.times[2]) {
            return this.velocity - (time - this.times[1]) * this.deceleration;
        } else {
            return 0;
        }
    }
}
