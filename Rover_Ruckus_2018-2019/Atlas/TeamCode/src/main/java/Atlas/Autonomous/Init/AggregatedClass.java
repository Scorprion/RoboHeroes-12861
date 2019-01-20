package Atlas.Autonomous.Init;

import android.graphics.Color;
import android.provider.CalendarContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static Atlas.Autonomous.Init.AggregatedClass.direction.CCW;
import static Atlas.Autonomous.Init.AggregatedClass.direction.CW;

public class AggregatedClass extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     * <p>
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like
     *
     * @param encoderDrive
     */

    //Using our robot's hardware
    public HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);
    //public static final double turnSpeed = 0.5;
    public boolean colorFound = false;

    protected double diffred = 0, diffblue = 0;
    protected int defaultRed = Color.red(67); //The default, constant red color value for our practice value
    protected int defaultBlue = Color.blue(86); //The default, constant red color value for our practice value

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

    private double error = 0;
    private double Poutput;
    private double Ioutput;
    private double Doutput;
    private double slope;
    private double pasterror = 0;
    private double angle = 0;
    protected Orientation sensor;
    private double output;
    private boolean atAngle = false;
    private boolean firstTime = true;
    protected void PIDCCW (double P, double I, double D, double target) {
        target /= 100;
        atAngle = false;
        Ioutput = 0;
        while (opModeIsActive() && !atAngle) {
            sensor = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(sensor.firstAngle);
            error = target - (angle / 100); // The target is our setpoint and our sensor is our IMU output

            Poutput = P * error;

            // The accumulator
            Ioutput = Ioutput + (I * error) / 100;

            // The slope is the "derivative" of the error
            if (firstTime) {
                slope = 0;
                Doutput = -D * slope;
                firstTime = false;
            } else {
                slope = pasterror - error;
                Doutput = -D * slope;
            }

            pasterror = error;

            // The output of the PID controller
            output = Poutput + Ioutput + Doutput;

            if (Math.round(error * 100) == 0) {
                atAngle = true;
            }

            if (output > 1) {
                output = 1;
            }

            robot.Left.setPower(output);
            robot.Right.setPower(-output);

            telemetry.addData("Output:", output);
            telemetry.addData("Target Angle:", target * 100);
            telemetry.addData("Current Angle:", normalizeAngle(sensor.firstAngle));
            telemetry.addData("Proportional:", Poutput);
            telemetry.addData("Integral:", Ioutput);
            telemetry.addData("Derivative:", Doutput);
            telemetry.addData("Error:", error);
            telemetry.update();

        }
        stopMotors();
        /*while(opModeIsActive()) {
            sensor = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Current Angle:", normalizeAngle(sensor.firstAngle));
            telemetry.update();

        } */
    }
    protected void PIDCW (double P, double I, double D, double target) {
        target /= 100;
        atAngle = false;
        Ioutput = 0;
        while(opModeIsActive() && !atAngle) {
            sensor = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(sensor.firstAngle);
            error = target - (angle / 100); // The target is our setpoint and our sensor is our IMU output

            Poutput = P * error;

            // The accumulator
            Ioutput = Ioutput + (I * error) / 100;

            // The slope is the "derivative" of the error
            if(firstTime) {
                slope = 0;
                Doutput = -D * slope;
                firstTime = false;
            } else {
                slope = pasterror - error;
                Doutput = -D * slope;
            }

            pasterror = error;

            // The output of the PID controller
            output = Poutput + Ioutput + Doutput;

            if(Math.round(error * 100) == 0) {
                atAngle = true;
            }

            if(output > 1) {
                output = 1;
            }

            robot.Left.setPower(-output);
            robot.Right.setPower(output);


            telemetry.addData("Output:", output);
            telemetry.addData("Target Angle:", target * 100);
            telemetry.addData("Current Angle:", normalizeAngle(sensor.firstAngle));
            telemetry.addData("Proportional:", Poutput);
            telemetry.addData("Integral:", Ioutput);
            telemetry.addData("Derivative:", Doutput);
            telemetry.addData("Error:", error);
            telemetry.update();
        }
        stopMotors();
        /*while(opModeIsActive()) {
            sensor = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Current Angle:", normalizeAngle(sensor.firstAngle));
            telemetry.update();

        }*/
    }




    public double normalizeAngle(double angle) {
        if (angle > -180 && angle < 0) {
            angle += 360;
        }
        return angle;
    }


    public void stopMotors() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    public void calibrateCS() {
        int cs = robot.ColorSensor.getNormalizedColors().toColor();
        defaultRed = Color.red(cs) - defaultRed;
        defaultBlue = Color.blue(cs) - defaultBlue;
    }

    /**
     * Does the movements based on if the gold was found in the first position
     *
     * @param program defines whether to drop our team marker in the depot or move aside (AC or AC2)
     */
    public void position1AC(int program) {
      /*sleep(400);
        proportional(0.5, 340, 3,5);
        telemetry.addLine("Found gold at 1");
        telemetry.update();
        encoderDrives(0.5, 4, 4); */
        sleep(400);
        encoderDrives(0.5, 7, 7);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 1");
        telemetry.update();
        sleep(500);
        encoderDrives(0.5, 18, 18);
        sleep(750);
        encoderDrives(0.5, 6, -6);
        sleep(250);
        encoderDrives(0.5, 4, 4);
        if (program == 1) {
            markerAC();
        } else {
            encoderDrives(0.5, -6, -6);
            encoderDrives(0.5, -8, 8);
            encoderDrives(0.5, 72, 72);
        }
    }

    public void position2AC(int program) {
       /* sleep(400);
        proportional(0.5, 360, 3,5);
        telemetry.addLine("Found gold at 2");
        telemetry.update();
        encoderDrives(0.5, 4, 4); */
        encoderDrives(0.5, 4, 4);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 2");
        telemetry.update();
        encoderDrives(0.5, 7, 7);
        if (program == 1) {
            markerAC();
        } else {
            encoderDrives(0.5, -6, -6);
            encoderDrives(0.5, -8, 8);
            encoderDrives(0.5, 72, 72);
        }
    }


    public void position3AC(int program) {
        sleep(500); // For debugging
        encoderDrives(0.5, 7, 7);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 3");
        telemetry.update();
        encoderDrives(0.5, 3, 3);
        encoderDrives(0.5, -4, 4);
        encoderDrives(0.5, 4, 4);
        if (program == 1) {
            markerAC();
        } else {
            encoderDrives(0.5, -6, -6);
            encoderDrives(0.5, -8, 8);
            encoderDrives(0.5, 72, 72);
        }
    }

    public void position1BD(int program) {
        sleep(400);
        encoderDrives(0.5, 6, 6);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 1");
        telemetry.update();
        encoderDrives(0.5, 4, 4);
        sleep(500);
        encoderDrives(0.5, -4, -4);
        encoderDrives(0.5, 12, -12);
        encoderDrives(0.5, 40, 40);
        encoderDrives(0.5, -15, 15);
        encoderDrives(0.8, 30, 30);
        if (program == 1) {
            markerBD();
        }
    }

    public void position2BD(int program) {
        sleep(400);
        encoderDrives(0.5, 7, 7);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 1");
        telemetry.update();
        encoderDrives(0.5, 4, 4);
        sleep(500);
        encoderDrives(0.5, -4, -4);
        encoderDrives(0.5, 12, -12);
        encoderDrives(0.5, 23, 23);
        encoderDrives(0.5, -15, 15);
        encoderDrives(0.8, 30, 30);
        if (program == 1) {
            markerBD();
        }
    }

    public void position3BD(int program) {
        sleep(400);
        encoderDrives(0.5, 7, 7);
        sleep(500);
        encoderDrives(0.5, 12, -12);
        telemetry.addLine("Found gold at 1");
        telemetry.update();
        encoderDrives(0.5, 4, 4);
        sleep(500);
        encoderDrives(0.5, -4, -4);
        encoderDrives(0.5, 12, -12);
        encoderDrives(0.5, 10, 10);
        encoderDrives(0.5, -15, 15);
        encoderDrives(0.8, 30, 30);
        if (program == 1) {
            markerBD();
        }
    }


    //TODO Finish this method
    /*public double inRange(double range1, double range2) {
        double range = range1 * countsPerInch;
        return range;
    }*/

    //Going to opposing crater
    public void markerAC2() {
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            NormalizedRGBA colors = robot.BottomCS.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                encoderDrives(0.5, 8, 8);
                sleep(200);
                robot.Marker.setPosition(0);
                encoderDrives(0.3, -6, -6);
                //proportional(CW, 0.3, 75, 2);
                encoderDrives(0.5, 15, -15);
                encoderDrives(1, 80, 75);
            }
        }
    }

    //Going to team crater
    public void markerAC() {
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            NormalizedRGBA colors = robot.BottomCS.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                    sleep(500);
                    encoderDrives(0.5, 8, 8);
                    sleep(500);
                    robot.Marker.setPosition(0);
                    encoderDrives(0.3, -8, -8);
                    //proportional(CW, 0.3, 75, 2);
                    encoderDrives(0.5, 15, -15);
                    encoderDrives(1, -69, -75);
            }
        }
    }

    public void markerBD() {
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(200);
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            NormalizedRGBA colors = robot.BottomCS.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
                sleep(500);
                //proportional(CW,0.5, 135, 2);
                encoderDrives(1, -75, -70);
            }
        }
    }
}

