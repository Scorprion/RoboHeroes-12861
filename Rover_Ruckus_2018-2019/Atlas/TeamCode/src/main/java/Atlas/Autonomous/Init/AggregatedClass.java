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

import java.util.concurrent.TimeUnit;

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
    public boolean markerFound = false;

    protected double diffred = 0, diffblue = 0;
    private int posCounter = 1;
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
            Ioutput = Ioutput + ((I * error) / 100);

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

    public void BD_CS2() {
        robot.runtime.reset();
        while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            int color = colors.toColor();

            diffred = 0; //colors.red - Color.red(defaultRed);
            diffblue = 0; //colors.blue - Color.blue(defaultBlue);


            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

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

        public void BD_CS() throws InterruptedException {
            robot.runtime.reset();
            while(opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
                NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
                int color = colors.toColor();

                diffred = 0; //colors.red - Color.red(defaultRed);
                diffblue = 0; //colors.blue - Color.blue(defaultBlue);

                float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                color = colors.toColor();

                telemetry.addData("Red:", colors.red);
                telemetry.addData("Calibrated Red:", diffred);
                telemetry.addData("Blue:", colors.blue + diffblue);
                telemetry.addData("Calibrated Blue:", diffblue);
                telemetry.update();
                if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                    if(posCounter == 1) {
                        telemetry.addLine("Gold found at 1");
                        telemetry.update();
                        colorFound = true;
                        middleBD();
                    } else if(posCounter == 2) {
                        telemetry.addLine("Gold found at 2");
                        telemetry.update();
                        colorFound = true;
                        leftBD();
                    } else {
                        telemetry.addLine("Gold found at 3");
                        telemetry.update();
                        colorFound = true;
                        rightBD();
                    }
                }
            }

        if(!colorFound && (robot.runtime.milliseconds() >= 500)) {
            posCounter++;
        }
    }

    public void AC_CS() throws InterruptedException {
        robot.runtime.reset();
        while(opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            int color = colors.toColor();

            diffred = 0; //colors.red - Color.red(defaultRed);
            diffblue = 0; //colors.blue - Color.blue(defaultBlue);


            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if(posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleAC();
                } else if(posCounter == 2) {
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

    public void AC_CS2() throws InterruptedException {
        robot.runtime.reset();
        while(opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            int color = colors.toColor();

            diffred = 0; //colors.red - Color.red(defaultRed);
            diffblue = 0; //colors.blue - Color.blue(defaultBlue);


            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();
            if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if(posCounter == 1) {
                    telemetry.addLine("Gold found at 1");
                    telemetry.update();
                    colorFound = true;
                    middleAC2();
                } else if(posCounter == 2) {
                    telemetry.addLine("Gold found at 2");
                    telemetry.update();
                    colorFound = true;
                    leftAC2();
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
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.3, -8, -8);
                sleep(100);
                encoderDrives(0.4, -13,13);
                sleep(100);
                encoderDrives(0.4, -15.5, -15.5);
                sleep(100);
                encoderDrives(0.5,3.5, -3.5);
                sleep(100);
                encoderDrives(1, -59, -59);
            }
        }
    }

    public void middleAC2() {
        robot.Left.setPower(-0.3); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.3);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.3, -8, -8);
                sleep(100);
                encoderDrives(0.4, 12, -12);
                sleep(100);
                encoderDrives(0.4, -19, -19);
                sleep(100);
                encoderDrives(1, -83, -80);
            }
        }
    }

    public void leftAC() {
            encoderDrives(0.3, 27, 27);
            sleep(100);
            encoderDrives(0.4, -4, -4);
            sleep(100);
            encoderDrives(0.4, 19, -9);
            sleep(100);
            encoderDrives(0.4, 6, 6);
            sleep(100);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 6, 6);
                sleep(100);
                robot.Marker.setPosition(0);
                encoderDrives(0.3, -8, -8);
                sleep(100);
                encoderDrives(0.4, -16,16);
                sleep(100);
                encoderDrives(0.4, -16, -16);
                sleep(100);
                encoderDrives(0.5, 3, -3);
                sleep(100);
                encoderDrives(1, -72, -72);
            }
        }
    }

    public void leftAC2() {
        encoderDrives(0.3, 27, 27);
        sleep(100);
        encoderDrives(0.4, -4, -4);
        sleep(100);
        encoderDrives(0.4, 15, -9);
        sleep(100);
        encoderDrives(0.4, 6, 6);
        sleep(100);
        telemetry.addLine("Started CSing");
        telemetry.update();
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(800);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.5, -12, -12);
                sleep(100);
                encoderDrives(0.5, 3.75, -3.75);
                sleep(100);
                encoderDrives(1, -76, -79);
            }
        }
    }

    public void rightAC() {
        encoderDrives(0.4, 22,22);
        sleep(100);
        encoderDrives(0.4, -12, 12);
        sleep(100);
        encoderDrives(0.4, 6, 5);

        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 8, 6);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.4, -6, -6);
                sleep(100);
                encoderDrives(0.3, -3, 3);
                sleep(100);
                encoderDrives(1, -78, -77);
            }
        }
    }

    public void rightAC2() {
        encoderDrives(0.4, 22,22);
        sleep(100);
        encoderDrives(0.4, -12, 12);
        sleep(100);
        encoderDrives(0.4, 6, 5);

        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 8, 6);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(0.5, -15, 15);
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                encoderDrives(0.5, 4, -4);
                sleep(100);
                encoderDrives(1, 74, 74);
            }
        }
    }

    public void middleBD() {
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
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(1, -76, -76);
            }
        }
    }

    public void middleBD2() {
        encoderDrives(0.4, 5, 5);
        sleep(100);
        encoderDrives(0.5, -4, 4);

    }

    public void leftBD() {
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
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(500);
                encoderDrives(1, -76, -76);
            }
        }
    }

    public void leftBD2() {
        encoderDrives(0.5, 8, 8);
        sleep(100);
    }

    public void rightBD() {
        encoderDrives(0.4, 8,8);
        sleep(100);
        encoderDrives(0.4, -14, -14);
        sleep(100);
        encoderDrives(0.4, -18, 18);
        sleep(100);
        encoderDrives(1, 44, 44);
        sleep(100);
        encoderDrives(0.5, -4, 4);
        sleep(100);
        encoderDrives(1, 35,35);
        robot.Left.setPower(-0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(700);
        while (opModeIsActive() && !markerFound) {
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
                sleep(100);
                encoderDrives(0.5, 8, 8);
                sleep(100);
                robot.Marker.setPosition(0);
                sleep(100);
                encoderDrives(1, -76, -76);
            }
        }
    }

    public void rightBD2() {
        encoderDrives(0.4, 8, 8);
        sleep(100);
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
                sleep(500);
                robot.Marker.setPosition(0);
                sleep(500);
                /*encoderDrives(0.3, -8, -8);
                //proportional(CW, 0.3, 75, 2);
                sleep(500);
                encoderDrives(1, -78, -77);*/
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
                    encoderDrives(0.4, -8,8);
                    encoderDrives(0.5,-0.4, 0.4);
                    encoderDrives(1, -78, -84);
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

