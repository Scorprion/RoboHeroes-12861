package Atlas.Autonomous.Init;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;

@SuppressWarnings({"unused", "WeakerAccess", "SameParameterValue"})
public class Controller extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     *
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like "encoderDrive"
     */

    //Using our robot's hardware
    public HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    private final double countsPerRot = 2240; // The counts per rotation
    private final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    private final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    protected final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);

    private float[] hsvValues = new float[3];
    private NormalizedRGBA colors = new NormalizedRGBA();
    private NormalizedRGBA colors2 = new NormalizedRGBA();
    //public static final double turnSpeed = 0.5;
    public boolean colorFound = false;
    private boolean markerFound = false;

    private double PIDout = 0, lasterror = 0;

    private double diffred = 0, diffblue = 0;
    private double diffred2 = 0, diffblue2 = 0;

    private double max = 0;
    private double max2 = 0;
    private int color = 0;
    private int color2 = 0;
    private int posCounter = 1;

    public enum direction {
        CW, CCW
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing
        robot.init(hardwareMap);
    }

    /**
     * Encoder method for controlling our autonomous programs
     *
     * @param speed   The speed at which the motors turn at
     * @param linches the distance for the left motor to turn (in inches)
     * @param rinches the distance for the right motor to turn (in inches)
     */
    public void encoderDrives(double speed,
                              double linches, double rinches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (-linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (-rinches * countsPerInch);

            //Both negative cw
            //Both positive backward
            //Left negative cw
            //Right negative backward
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.Left.getCurrentPosition(),
                        robot.Right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Left.setPower(0);
            robot.Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrives(double speed,
                              double linches, double rinches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (-linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (-rinches * countsPerInch);
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Resetting the time and setting the power of the motors
            robot.runtime.reset();
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.runtime.seconds() < timeoutS) &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.Left.getCurrentPosition(),
                        robot.Right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Left.setPower(0);
            robot.Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void stopMotors() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
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
    public double eulerNormalize(double value) {
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

    // Have to throw IOException in order to not get an error
    protected void PID(double P, double I, double D, double target) throws IOException {

        DataLogger d = new DataLogger("Test.csv");
        d.addHeaderLine("Iteration, Time, Proportional, Integral, Total Output, Error, " +
                "Current, Angular Velocity, Angular Acceleration, Setpoint");

        int iteration = 0;
        double dt = 0, ang_velocity = 0, ang_accel = 0, total_time = 0;
        lasterror = 0;
        PIDout = 0;
        slope = 1;

        double angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
        error = getError(target, angle);
        while(opModeIsActive() && !errorCheckStop(error, 2, 1)) {
            time.reset();
            angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
            output = calcPID(P, I, D, target, angle, dt);
            output /= 100;
            error = getError(target, angle);

            telemetry.addData("Output: ", output);
            output = constrain(output, -1, 1);

            iteration++;
            dt = time.seconds();
            ang_velocity = (eulerNormalize(robot.imu.getAngularOrientation().firstAngle) - angle) / dt;
            ang_accel = ang_velocity / dt;
            total_time += dt;

            robot.Right.setPower(-output);
            robot.Left.setPower(output);
            telemetry.addData("Iteration", iteration);
            telemetry.addData("Normalized Output: ", output);
            telemetry.addData("Proportional: ", getP());
            telemetry.addData("Integral: ", getI());
            telemetry.addData("Derivative: ", getD());
            telemetry.addData("Error: ", getError(target, angle));
            telemetry.addData("Velocity", ang_velocity);
            telemetry.addData("Current Angle: ", angle);
            telemetry.update();

            d.addDataLine(iteration + "," + total_time + "," + getP() + "," + getI() + "," + output + "," + error +
                    "," + angle + "," + ang_velocity + "," + ang_accel + "," + target);
        }

        d.close();
        stopMotors();
        telemetry.addData("Error stop: ", errorCheckStop(error, 1, 0));
        telemetry.addData("Not moving: ", notMoving(output, 2, 0.14));
        telemetry.addData("Current Angle: ", angle);
        telemetry.update();
    }

    private double calcPID(double P, double I, double D, double target, double sensor, double dt) {
        error = target - sensor;
        error = eulerNormalize(error);

        Poutput = P * error;
        Poutput = constrain(Poutput, -30, 30);

        Ioutput += I * error * dt;

        slope = error - lasterror;
        Doutput = -D * slope / dt;

        lasterror = error;

        PIDout = Poutput + Ioutput + Doutput;

        return PIDout;
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
    protected double round(double value, int places) {
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

