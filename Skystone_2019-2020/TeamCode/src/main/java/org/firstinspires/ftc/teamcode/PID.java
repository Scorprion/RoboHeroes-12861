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
    public double slope = 0;
    public double parabola = 0;
    public double PIDout, Poutput, Ioutput, Doutput;

    public double P, I, D, setpoint, lasterror, total_error;

    public PID(double P, double I, double D, double setpoint) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.setpoint = setpoint;  // The target angle
        this.lasterror = 0;  // For the derivative part of the calculation
        this.error = 0;
    }

    public double getPID(double current_angle) {
        PIDout = calcPID(current_angle);
        return PIDout;
    }

    private double calcPID(double angle) {
        this.error = (likeallelse(angle) - this.setpoint) / 360;

        // Proportional
        Poutput = P * this.error;

        // Integral
        Ioutput = (I * this.total_error) / 100; // / time.seconds();

        // Derivative
        this.slope = this.error - this.lasterror;
        Doutput = -D * this.slope;

        // Storing the saved error value for the derivative calculation later
        this.lasterror = this.error;
        this.total_error += error;
        PIDout = Poutput + Ioutput + Doutput;

        return PIDout;
    }

    public void setParams(double P, double I, double D, double setpoint) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.setpoint = setpoint;  // The target angle
        this.lasterror = 0;  // For the derivative part of the calculation
        this.error = 0;  // Resetting the PID
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
     * 3 notMoving boolean methods that return true of false based upon their given parameters. For example, the 1st
     * notMoving method returns true if the movement rounded to the nearest place is equal to 0
     * @param movement the movement value to be evaluated
     * @param place the decimal place to round to
     * @param underThresh the number that is checked against the value to see if the rounded number is less
     *                    (or under) that threshold (when rounding to a decimal point beyond whole numbers)
     * @return true or false based upon if the movement value rounded to the placeth is less than the range passed
     */
    private boolean notMoving(double movement, int place, double underThresh) {
        return inrange(round(movement, place), underThresh);
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
     * Check to see if we should stop the program based on if the rounded error is equal to 0
     * @param error the error
     * @param place the place to round to
     * @return a boolean of True of False
     */
    private boolean errorCheckStop(double error, int place, double underThresh) {
        /*Example of refactoring
        error = round(error, place);
        return((error == 0)? true: false);*/
        return inrange(round(error, place), underThresh);
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

    /**
     * Returns True of False, depending on if the value is under or equal to the range
     * @param value the value to be checked
     * @param under the value the value has to be under (less than)
     * @return True of False, depending on if the value is in range of the under value
     */
    private boolean inrange(double value, double under) {
        return Math.abs(value) < under || value == under;
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

    private double likeallelse(double angle) {
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

    /**
     * Retrieving the error
     * @return the error
     */
    public double getError() {
        return error;
    }

    /**
     * Retrieve the proportional output
     * @return the Poutput
     */
    public double getP() {
        return Poutput;
    }

    /**
     * Retrieve the integral output
     * @return the Ioutput
     */
    public double getI() {
        return Ioutput;
    }

    /**
     * Retrieve the derivative output
     * @return the Doutput
     */
    public double getD() {
        return Doutput;
    }

    protected void setSetpoint(double value) {
        setpoint = value;
    }
}
