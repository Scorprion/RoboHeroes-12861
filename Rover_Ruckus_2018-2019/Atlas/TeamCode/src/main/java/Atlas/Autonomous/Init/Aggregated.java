package Atlas.Autonomous.Init;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.lang.Math.*;

@SuppressWarnings({"unused", "WeakerAccess", "SameParameterValue"})
public class Aggregated extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     *
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like "encoderDrive"
     */

    //Using our robot's hardware
    HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    private final double countsPerRot = 2240; // The counts per rotation
    private final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    private final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);

    private float[] hsvValues = new float[3];
    private NormalizedRGBA colors = new NormalizedRGBA();
    //public static final double turnSpeed = 0.5;
    public boolean colorFound = false;
    private boolean markerFound = false;

    private double PIDout = 0, lasterror = 0, total_error = 0;

    private double diffred = 0, diffblue = 0;
    private double max = 0;
    private int color = 0;
    private int posCounter = 1;
    private int defaultRed = Color.red(67); //The default, constant red color value for our practice value
    private int defaultBlue = Color.blue(86); //The default, constant red color value for our practice value

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

    public void calibrateCS() {
        int cs = robot.ColorSensor.getNormalizedColors().toColor();
        defaultRed = Color.red(cs) - defaultRed;
        defaultBlue = Color.blue(cs) - defaultBlue;
    }

    protected void BD_CS2() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            colors = robot.ColorSensor.getNormalizedColors();
            color = colors.toColor();
            max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            diffred = 0; //defaultRed - colors.red;
            diffblue = 0; //defaultBlue - colors.blue;

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if (Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if (posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleBD2();
                } else if (posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftBD2();
                } else {
                    telemetry.addLine("Gold found at 3");
                    telemetry.update();
                    colorFound = true;
                    rightBD2();
                }
            }
        }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    protected void BD_CS() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            colors = robot.ColorSensor.getNormalizedColors();
            color = colors.toColor();
            max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            diffred = 0; //defaultRed - colors.red;
            diffblue = 0; //defaultBlue - colors.blue;

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if (posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleBD();
                } else if (posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftBD();
                }
            }
        }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    protected void DelayedBD_CS() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            colors = robot.ColorSensor.getNormalizedColors();
            color = colors.toColor();
            max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            diffred = 0; //defaultRed - colors.red;
            diffblue = 0; //defaultBlue - colors.blue;

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if (posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    DelayedmiddleBD();
                } else if (posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftBD();
                }
            }
        }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    protected void AC_CS() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            colors = robot.ColorSensor.getNormalizedColors();
            color = colors.toColor();
            max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            diffred = 0; //defaultRed - colors.red;
            diffblue = 0; //defaultBlue - colors.blue;

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if (Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if (posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleAC();
                } else if (posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftAC();
                } else {
                    telemetry.addLine("Gold found at 3");
                    telemetry.update();
                    colorFound = true;
                    rightAC();
                }
            }
        }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    protected void AC_CS2() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            colors = robot.ColorSensor.getNormalizedColors();
            color = colors.toColor();
            max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            diffred = 0; //defaultRed - colors.red;
            diffblue = 0; //defaultBlue - colors.blue;

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if (Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if (posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleAC2();
                } else if (posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftAC2();
                } else {
                    telemetry.addLine("Gold found at 3");
                    telemetry.update();
                    colorFound = true;
                    rightAC2();
                }
            }
        }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    /**
     * Does the movements based on if the gold was found in the first position
     */
    public void middleAC() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 6, 6);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            float[] hsvValues = new float[3];
            NormalizedRGBA colors = robot.BottomCS.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(0.3,  8, 8);
                sleep(100);
                encoderDrives(0.4, -10,10);
                sleep(100);
                encoderDrives(1, -45, -45);
                sleep(100);
                encoderDrives(0.2, -30, -30);
                LatchReset();

            }
        }
    }

    public void middleAC2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Left.setPower(-0.3); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.3);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(0.3, -5, -5);
                sleep(100);
                encoderDrives(0.4, -12, 12);
                sleep(100);
                encoderDrives(0.4, 19, 19);
                sleep(100);
                encoderDrives(0.4, -3, 3);
                sleep(100);
                encoderDrives(1, 50,50);
                sleep(100);
                encoderDrives(0.05, 5, 5, 3);
                LatchReset();
            }
        }
    }

    public void leftAC() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.3, 27, 27);
        sleep(100);
        encoderDrives(0.4, -4, -4);
        sleep(100);
        encoderDrives(0.4, 10, -10);
        sleep(100);
        encoderDrives(0.4, 6, 6);
        sleep(100);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.3, 8, 8);
                sleep(100);
                encoderDrives(0.4, -16,16);
                sleep(100);
                encoderDrives(0.4, -16, -16);
                sleep(100);
                encoderDrives(1, -50,-50);
                sleep(100);
                encoderDrives(0.08, -13 ,-13);
                LatchReset();

            }
        }
    }

    public void leftAC2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.3, 27, 27);
        sleep(100);
        encoderDrives(0.4, -4, -4);
        sleep(100);
        encoderDrives(0.4, 11, -11);
        sleep(100);
        encoderDrives(0.4, 4, 4);
        sleep(100);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(0.4, -20, -20);
                sleep(100);
                encoderDrives(0.4, 27, -27);
                sleep(100);
                encoderDrives(0.5, 32, 32);
                sleep(100);
                encoderDrives(0.1, 15, 15, 5);
                LatchReset();

            }
        }
    }

    public void rightAC() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 22,22);
        sleep(100);
        encoderDrives(0.4, -12, 12);
        sleep(100);
        encoderDrives(0.4, 6, 5);
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(0.4, -6, -6);
                sleep(100);
                encoderDrives(0.3, -4, 4);
                sleep(100);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.08, -12,-12);
                LatchReset();
            }
        }
    }

    public void rightAC2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 22,22);
        sleep(100);
        encoderDrives(0.4, -12, 12);
        sleep(100);
        encoderDrives(0.4, 6, 5);

        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(0.5, -9, 9);
                sleep(100);
                encoderDrives(0.5, 15, 15);
                sleep(100);
                encoderDrives(0.4, -3, 3);
                sleep(100);
                encoderDrives(1, 57, 57);
                sleep(100);
                encoderDrives(0.1, 12, 12, 7);
                LatchReset();
            }
        }
    }

    public void middleBD() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 5, 5);
        sleep(100);
        encoderDrives(0.4, -8, -8);
        sleep(100);
        encoderDrives(0.4, -11.25, 11.25);
        sleep(100);
        encoderDrives(0.4, 40, 40);
        sleep(100);
        encoderDrives(0.4, -4, 4);
        sleep(100);
        encoderDrives(0.4, 28, 28);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.1, -15, -51);
                LatchReset();
            }
        }
    }

    public void DelayedmiddleBD() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 5, 5);
        sleep(100);
        encoderDrives(0.4, -8, -8);
        sleep(4000);
        encoderDrives(0.4, -11.25, 11.25);
        sleep(100);
        encoderDrives(0.4, 40, 40);
        sleep(100);
        encoderDrives(0.4, -4, 4);
        sleep(100);
        encoderDrives(0.4, 28, 28);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.1, -15, -51);
                LatchReset();
            }
        }
    }

    public void middleBD2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 5, 5);
        sleep(100);
        encoderDrives(0.5, -4, 4);
        LatchReset();

    }

    public void leftBD() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 7, 7);
        sleep(100);
        encoderDrives(0.4, -14, -14);
        sleep(100);
        encoderDrives(0.4, -6, 6);
        sleep(100);
        encoderDrives(0.4, 35, 35);
        sleep(100);
        encoderDrives(0.5, -6, 6);
        sleep(100);
        encoderDrives(0.4, 28, 28);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(500);
                sleep(100);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.1, -15,-15);
                LatchReset();
            }
        }
    }

    public void leftBD2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.5, 8, 8);
        sleep(100);
    }

    public void rightBD() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderDrives(0.4, 8,8);
        sleep(100);
        encoderDrives(0.4, -14, -14);
        sleep(100);
        encoderDrives(0.4, -18, 18);
        sleep(100);
        encoderDrives(1, 40, 40);
        sleep(100);
        encoderDrives(0.5, -4, 4);
        sleep(100);
        encoderDrives(1, 35,35);
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                sleep(100);
                encoderDrives(1, -50, -50);
                encoderDrives(0.1, -15,-15);
                LatchReset();
            }
        }
    }

    public void rightBD2() {
        robot.LShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(100);
        encoderDrives(0.4, -4, 4);
        sleep(100);
        encoderDrives(0.4, 8, 8);
        sleep(100);
        LatchReset();
    }

    protected void position1BD(int program) {
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

    protected void position2BD(int program) {
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

    protected void position3BD(int program) {
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
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
                sleep(500);
                encoderDrives(0.3, -6, 6);
                sleep(500);
                encoderDrives(1, 50, 50);
                sleep(100);
                encoderDrives(0.3, 28, 22);
            }
        }
    }

    //Going to team crater
    //Note: this method is the same program for all AC paths, whether the gold is in the center,
    //left, or the right
    public void markerAC() {
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
                encoderDrives(0.3, -8, -8);
                sleep(100);
                encoderDrives(0.4, -8,8);
                sleep(100);
                encoderDrives(0.5,-0.4, 0.4);
                sleep(100);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.3, -28, -34);
            }
        }
    }

    private void markerBD() {
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(200);
        while (opModeIsActive() && !colorFound) {
            initColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
                sleep(500);
                encoderDrives(1, -50, -50);
                sleep(100);
                encoderDrives(0.3, -25, -20);
            }
        }
    }

    public void LatchReset() {
        robot.Sliding.setPosition(0);
        robot.Latching.setPower(-1);
        sleep(1200);
        robot.Latching.setPower(0);
    }

    private void initColor() {
        NormalizedRGBA colors = robot.BottomCS.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        colors.toColor();
        max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        color = colors.toColor();
    }
}

