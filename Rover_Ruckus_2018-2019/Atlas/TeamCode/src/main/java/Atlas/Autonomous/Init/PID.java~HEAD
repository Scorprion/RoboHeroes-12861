package Atlas.Autonomous.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class PID {

    public PID(double P, double I, ) {

    }

    //Methods for testing
    /**
     * Constrain a given value between 2 values, useful for things like power which can only be between 1 and -1
     * @param value the value to be costrained
     * @param min the minimum for the value to be constrained around
     * @param max the maximum for the value to be constrained around
     * @return the constrained value
     */
    private double constrain(double value, double min, double max) {
        if(value > max) {
            return max;
        }
        else if(value < min) {
            return min;
        }

        return value;
    }

    /**
     * Normalize a value around the euler angles' values (-179 to 179 in 2 degrees for the IMU sensor)
     * @param value the value to be normalized
     * @return the normalized version of the double "value"
     */
    protected double eulerNormalize(double value) {
        while(value > 180) {
            value -= 360;
        }

        while(value < -180) {
            value += 360;
        }
        return value;
    }

    private double error = 0, slope, Poutput, Ioutput, Doutput, output;
    ElapsedTime time = new ElapsedTime();
    protected void PID(double P, double I, double D, double target) {
        int iteration = 0;
        lasterror = 0;
        PIDout = 0;
        slope = 1;
        double angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
        error = getError(target, angle);
        time.reset();

        while(opModeIsActive() && !errorCheckStop(error, 2, 1)) {
            angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
            output = tanh(calcPID(P, I, D, target, angle, time));
            error = getError(target, angle);

            telemetry.addData("Output: ", output);
            // output = constrain(output, -1, 1);

            iteration++;

            robot.Right.setPower(-output);
            robot.Left.setPower(output);
            telemetry.addData("Iteration", iteration);
            telemetry.addData("Normalized Output: ", output);
            telemetry.addData("Proportional: ", getP());
            telemetry.addData("Integral: ", getI());
            telemetry.addData("Derivative: ", getD());
            telemetry.addData("Error: ", getError(target, angle));
            telemetry.addData("Current Angle: ", angle);
            telemetry.update();
        }

        stopMotors();
        telemetry.
                addData("Error stop: ", errorCheckStop(error, 1, 0));
        telemetry.addData("Not moving: ", notMoving(output, 2, 0.14));
        telemetry.addData("Current Angle: ", angle);
        telemetry.update();
    }

    private double calcPID(double P, double I, double D, double target, double sensor, ElapsedTime time) {
        error = target - sensor;
        error = eulerNormalize(error);

        // Proportional
        Poutput = P * error;

        // Integral
        Ioutput += I/time.seconds() * total_error;

        // Derivative
        slope = error - lasterror;
        Doutput = -D * slope;

        // Storing the saved error value for the derivative calculation later
        lasterror = error;
        total_error += error;

        PIDout = Poutput + Ioutput + Doutput;

        return PIDout;
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
     * Retrieving the error
     * @return the error
     */
    protected double getError(double target, double sensor) {
        return target - sensor;
    }

    /**
     * Retrieve the proportional output
     * @return the Poutput
     */
    protected double getP() {
        return Poutput;
    }

    /**
     * Retrieve the integral output
     * @return the Ioutput
     */
    protected double getI() {
        return Ioutput;
    }

    /**
     * Retrieve the derivative output
     * @return the Doutput
     */
    protected double getD() {
        return Doutput;
    }
}
