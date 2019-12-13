package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;

@SuppressWarnings({"WeakerAccess"})
public class PID {
    public double total_angle = 0, previous_angle = 0, delta_angle = 0;

    public double angle = 0;
    public double error = 0;
    public double delta_error = 0;
    public double delta_time;
    public double parabola = 0;

    public double PIDout, Poutput, Ioutput, Doutput;
    public double P, I, D, setpoint, lasterror, time, lasttime, iminmax;

    public ElapsedTime timer;

    public PID(double P, double I, double D, double setpoint, Double iMinMax) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.setpoint = setpoint;  // The target angle
        this.lasterror = 0;  // For the derivative part of the calculation
        this.error = 0;
        this.time = 0;
        this.lasttime = 0;
        this.timer = new ElapsedTime();

        this.iminmax = iMinMax == null ? Double.POSITIVE_INFINITY : iMinMax.floatValue();  // An optional param

        this.Poutput = 0;
        this.Ioutput = 0;
        this.Doutput = 0;
    }

    public double getPID(double error) {
        PIDout = calcPID(error);
        return PIDout;
    }

    private double calcPID(double error) {
        //          Convert back to seconds, just more accurate
        this.time = this.timer.milliseconds() * 1000;
        this.error = error;

        // Proportional
        this.Poutput = P * this.error;


        delta_time = this.time - this.lasttime;
        delta_error = this.error - this.lasterror;

        // Integral
        this.Ioutput += error * this.delta_time;

        if(this.Ioutput > this.iminmax) {
            this.Ioutput = this.iminmax;
        } else if(this.Ioutput < -this.iminmax) {
            this.Ioutput = -this.iminmax;
        }

        // Derivative
        this.Doutput = -D * (delta_error / delta_time);

        // Storing the saved error value for the derivative calculation later
        this.lasterror = this.error;
        this.lasttime = this.time;
        PIDout = Poutput + I * Ioutput + Doutput;

        return PIDout;
    }

    public void setParams(double P, double I, double D, double setpoint, Double lasterror) {
        this.timer.reset();
        this.time = timer.milliseconds() * 1000;
        this.error = 0;

        this.lasterror = lasterror == null ? 0 : lasterror;  // For the derivative part of the calculation
        this.lasttime = 0;

        this.P = P;
        this.I = I;
        this.D = D;
        this.setpoint = setpoint;  // The target angle
    }

    public boolean Incomp(double ParPos, double ParNeg){
        return (delta_angle >= ParPos || delta_angle <= ParNeg);
    }

    /**
     * A tanh function to normalize values from -1 to 1
     * @param x the number to be normalized
     * @return the value of the tanh function
     */
    public static double tanh(double x) {
        return Math.tanh(x);
    }

    /**
     * A sigmoid function to normalize values from 0 to 1
     * @param x the number to be normalized
     * @return the value of the sigmoid function
     */
    public double sigmoid(double x) {
        return (1/( 1 + Math.pow(Math.E,(-1*x))));
    }

    /**
     * Another notMoving boolean method
     * @param movement the movement value to be evaluated
     * @param threshold the threshold value that the movement number cannot be less than
     * @return true or false based on if the movement value is less than the threshold value
     */
    private boolean notMoving(double movement, double threshold) {
        return movement < threshold;
    }

    /**
     * The 3rd notMoving boolean method
     * @param movement the movement value that can't be less than 0.05 or else the simulated robot isn't moving anymore
     * @return true of false based on if movement is less than 0.05 (5%)
     */
    private boolean notMoving(double movement) {
        return movement < 0.05;
    }

    /**
     * Check to see if the "round_value" is "close enough" to "value"
     * @param round_value the value to check
     * @param place the place to round to
     * @param value the value to compare the rounded value to
     * @return close enough
     */
    public boolean closeEnoughTo(double round_value, int place, double value) {
        /*Example of refactoring
        error = round(error, place);
        return((error == 0)? true: false);*/
        return round(round_value, place) == value;
    }

    /**
     * An accurate way of rounding a specific decimal place courtesy of Jonik
     * @link https://stackoverflow.com/questions/2808535/round-a-double-to-2-decimal-places
     * @param value the value to be rounded
     * @param places the nth place to round to
     * @return the rounded value
     */
    private double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_DOWN);
        return bd.doubleValue();
    }


    private boolean inrange(double value, double max, double min) {
        return value <= max && value >= min;
    }

    /**
     * Constrain a given value between 2 values, useful for things like power which can only be between 1 and -1
     * @param value the value to be costrained
     * @param min the minimum for the value to be constrained around
     * @param max the maximum for the value to be constrained around
     * @return the constrained value
     */
    private double constrain(double value, double min, double max) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        }
        return value;
    }

    public double likeallelse(double angle) {
        this.delta_angle = angle - previous_angle;

        if (delta_angle < -180)
            delta_angle += 360;
        else if (delta_angle > 180)
            delta_angle -= 360;

        this.total_angle += delta_angle;
        previous_angle = angle;
        return total_angle;
    }

    /*
    Shift all the values to a parabola with the minimum at your desired target

    private double parabolaShift(double angle, double target) {
        if(angle >= 0) {
            error = angle - target;
        } else if(angle < 0) {
            error = (angle + 180) - target;
        }
        return this.parabola / 180;
    }*/

    public void update_error(double error) {
        this.error = error;
    }
}
