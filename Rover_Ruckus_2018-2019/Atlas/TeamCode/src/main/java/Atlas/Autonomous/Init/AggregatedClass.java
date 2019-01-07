package Atlas.Autonomous.Init;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.sql.Time;

import Atlas.Autonomous.Init.HardwareAtlas;

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
    protected HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    protected final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);
    //public static final double turnSpeed = 0.5;
    public boolean colorFound = false;


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
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (rinches * countsPerInch);
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

    //Can turn up to 4 degrees accurately
    public void proportional(double turnSpeed, double targetAngle, int corrections, int TimeoutSec) {
        if (targetAngle < 0) {
            targetAngle = 360;
        }

        while (Math.abs(targetAngle) >= 360) {
            targetAngle = Math.abs(targetAngle) - 360;
        }
        robot.runtime.reset(); //Resetting timer

        //In case we accidentally add a timeout for 30 seconds or longer
        if (TimeoutSec >= 30) {
            telemetry.addLine("The time out is currently greater than or equal to 30 seconds, you might need to change something");
            telemetry.update();
        }

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        optimizeTurn(normalizeAngle(robot.angles.firstAngle), targetAngle, turnSpeed, corrections, TimeoutSec);
        /*double newTurnSpeed = 0;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = normalizeAngle(robot.angles.firstAngle);
        boolean loop = true;
        while(opModeIsActive() && loop) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(robot.angles.firstAngle);
            if(angle >= targetAngle) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                loop = false;
            }
        }

        for(int times = 0; corrections > times && robot.angles.firstAngle != targetAngle; times++) {
            newTurnSpeed = getTurnSpeed(turnSpeed, times);
            angle = normalizeAngle(robot.angles.firstAngle);
            int s = assignSign(newTurnSpeed, angle, targetAngle);
            turnOtherway(newTurnSpeed, angle, targetAngle);
            if(checkSurpassed(s, targetAngle)) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                telemetry.addData("Error:", getError(targetAngle, normalizeAngle(robot.angles.firstAngle)));
                telemetry.addData("Speed:", newTurnSpeed);
                telemetry.addData("Sign:", s);
                telemetry.addData("Angle:", angle);
                telemetry.update();
            }
        }

        sleep(20000);*/
    }


    //Turns ccw or cw based on which way it passed the target angle
    public void turnOtherway(double newTurnSpeed, double angle, double targetAngle) {
        if (angle > targetAngle) {
            robot.Left.setPower(-newTurnSpeed);
            robot.Right.setPower(newTurnSpeed);
        } else {
            robot.Left.setPower(newTurnSpeed);
            robot.Right.setPower(-newTurnSpeed);
        }
    }

    //Convert the angle from -179 and 180 degrees to 0 and 360 degrees
    public double normalizeAngle(double angle) {
        if (angle > -180 && angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public double getTurnSpeed(double s, int r) {
        //s /= Math.pow(2, r);
        s /= 2 * (r + 1);
        return s;
    }

    public int assignSign(double angle, double targetAngle) {
        int sign = 1;
        if (angle >= targetAngle) {
            return -1;
        }
        return sign;
    }

    public boolean checkSurpassed(int sign, double targetAngle, double TimeoutSec) {
        boolean loop = true;
        while (loop && opModeIsActive() && robot.runtime.seconds() < TimeoutSec) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = normalizeAngle(robot.angles.firstAngle);
            telemetry.addData("Error:", getError(targetAngle, angle));
            telemetry.addData("Angle:", angle);
            telemetry.update();
            if (sign == 1) {
                if (angle > targetAngle) {
                    robot.Left.setPower(0);
                    robot.Right.setPower(0);
                    loop = false;
                    return true;
                }
            } else {
                if (angle < targetAngle) {
                    robot.Left.setPower(0);
                    robot.Right.setPower(0);
                    loop = false;
                    return true;
                }
            }
        }
        return true;
    }

    public double getError(double tangle, double cangle) {
        return tangle - cangle;
    }

    public void ccw(double turnSpeed, double targetAngle, int corrections, double TimeoutSec) {
        double newTurnSpeed = 0;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = normalizeAngle(robot.angles.firstAngle);
        boolean loop = true;
        while (opModeIsActive() && loop && robot.runtime.seconds() < TimeoutSec) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(robot.angles.firstAngle);
            if (angle <= targetAngle) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                loop = false;
            }
        }

        for (int times = 0; corrections > times && robot.angles.firstAngle != targetAngle
                && robot.runtime.seconds() < TimeoutSec; times++) {
            newTurnSpeed = getTurnSpeed(turnSpeed, times);
            angle = normalizeAngle(robot.angles.firstAngle);
            int s = assignSign(angle, targetAngle);
            turnOtherway(newTurnSpeed, angle, targetAngle);
            if (checkSurpassed(s, targetAngle, TimeoutSec)) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                telemetry.addData("Corrections:", corrections);
                telemetry.addData("Error:", getError(targetAngle, normalizeAngle(robot.angles.firstAngle)));
                telemetry.addData("Speed:", newTurnSpeed);
                telemetry.addData("Sign:", s);
                telemetry.addData("Angle:", angle);
                telemetry.update();
            }
        }
    }

    public void cw(double turnSpeed, double targetAngle, int corrections, double TimeoutSec) {
        double newTurnSpeed = 0;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = normalizeAngle(robot.angles.firstAngle);
        boolean loop = true;
        while (opModeIsActive() && loop && robot.runtime.seconds() < TimeoutSec) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(robot.angles.firstAngle);
            if (angle >= targetAngle) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                loop = false;
            }
        }

        for (int times = 0; corrections > times && robot.angles.firstAngle != targetAngle
                && robot.runtime.seconds() < TimeoutSec; times++) {
            newTurnSpeed = getTurnSpeed(turnSpeed, times);
            angle = normalizeAngle(robot.angles.firstAngle);
            int s = assignSign(angle, targetAngle);
            turnOtherway(newTurnSpeed, angle, targetAngle);
            if (checkSurpassed(s, targetAngle, TimeoutSec)) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                telemetry.addData("Corrections:", corrections);
                telemetry.addData("Error:", getError(targetAngle, normalizeAngle(robot.angles.firstAngle)));
                telemetry.addData("Speed:", newTurnSpeed);
                telemetry.addData("Sign:", s);
                telemetry.addData("Angle:", angle);
                telemetry.update();
            }
        }
    }

    public void optimizeTurn(double currentAngle, double targetAngle, double speed, int corrections, double TimeoutSec) {
        if (currentAngle == 0) {
            currentAngle = 360;
        }
        double tc = targetAngle - currentAngle, ct = currentAngle - targetAngle;

        //TODO Fix the bug where it sometimes chooses the inefficient route
        if (tc < ct) {
            telemetry.addData("Needs to go clockwise:", Math.abs(tc));
            telemetry.update();
            robot.Left.setPower(speed);
            robot.Right.setPower(-speed);
            cw(speed, targetAngle, corrections, TimeoutSec);
        } else if (tc > ct) {
            telemetry.addData("Needs to go counter clockwise:", Math.abs(tc));
            telemetry.update();
            robot.Left.setPower(-speed);
            robot.Right.setPower(speed);
            ccw(speed, targetAngle, corrections, TimeoutSec);
        }
    }


    /**
     * Added alternative method for using proportional control in case we don't want to use a timeout
     */
    public void proportional(double turnSpeed, double targetAngle, int corrections) {
        if (targetAngle < 0) {
            targetAngle = 360;
        }

        while (Math.abs(targetAngle) >= 360) {
            targetAngle = Math.abs(targetAngle) - 360;
        }

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        optimizeTurn(normalizeAngle(robot.angles.firstAngle), targetAngle, turnSpeed, corrections);
    }

    public boolean checkSurpassed(int sign, double targetAngle) {
        boolean loop = true;
        while (loop && opModeIsActive()) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = normalizeAngle(robot.angles.firstAngle);
            telemetry.addData("Error:", getError(targetAngle, angle));
            telemetry.addData("Angle:", angle);
            telemetry.update();
            if (sign == 1) {
                if (angle > targetAngle) {
                    robot.Left.setPower(0);
                    robot.Right.setPower(0);
                    loop = false;
                    return true;
                }
            } else {
                if (angle < targetAngle) {
                    robot.Left.setPower(0);
                    robot.Right.setPower(0);
                    loop = false;
                    return true;
                }
            }
        }
        return true;
    }

    public void ccw(double turnSpeed, double targetAngle, int corrections) {
        double newTurnSpeed = 0;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = normalizeAngle(robot.angles.firstAngle);
        boolean loop = true;
        while (opModeIsActive() && loop) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(robot.angles.firstAngle);
            if (angle <= targetAngle) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                loop = false;
            }
        }

        for (int times = 0; corrections > times && robot.angles.firstAngle != targetAngle; times++) {
            newTurnSpeed = getTurnSpeed(turnSpeed, times);
            angle = normalizeAngle(robot.angles.firstAngle);
            int s = assignSign(angle, targetAngle);
            turnOtherway(newTurnSpeed, angle, targetAngle);
            if (checkSurpassed(s, targetAngle)) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                telemetry.addData("Corrections:", corrections);
                telemetry.addData("Error:", getError(targetAngle, normalizeAngle(robot.angles.firstAngle)));
                telemetry.addData("Speed:", newTurnSpeed);
                telemetry.addData("Sign:", s);
                telemetry.addData("Angle:", angle);
                telemetry.update();
            }
        }
    }

    public void cw(double turnSpeed, double targetAngle, int corrections) {
        double newTurnSpeed = 0;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = normalizeAngle(robot.angles.firstAngle);
        boolean loop = true;
        while (opModeIsActive() && loop) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = normalizeAngle(robot.angles.firstAngle);
            if (angle >= targetAngle) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                loop = false;
            }
        }

        for (int times = 0; corrections > times && robot.angles.firstAngle != targetAngle; times++) {
            newTurnSpeed = getTurnSpeed(turnSpeed, times);
            angle = normalizeAngle(robot.angles.firstAngle);
            int s = assignSign(angle, targetAngle);
            turnOtherway(newTurnSpeed, angle, targetAngle);
            if (checkSurpassed(s, targetAngle)) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                telemetry.addData("Corrections:", corrections);
                telemetry.addData("Error:", getError(targetAngle, normalizeAngle(robot.angles.firstAngle)));
                telemetry.addData("Speed:", newTurnSpeed);
                telemetry.addData("Sign:", s);
                telemetry.addData("Angle:", angle);
                telemetry.update();
            }
        }
    }

    public void optimizeTurn(double currentAngle, double targetAngle, double speed, int corrections) {
        double tc = targetAngle - currentAngle;

        if (Math.abs(tc) > 180) {
            telemetry.addData("Needs to go clockwise:", Math.abs(tc));
            telemetry.update();
            robot.Left.setPower(speed);
            robot.Right.setPower(-speed);
            cw(speed, targetAngle, corrections);
        } else if (Math.abs(tc) <= 180) {
            telemetry.addData("Needs to go counter clockwise:", Math.abs(tc));
            telemetry.update();
            robot.Left.setPower(-speed);
            robot.Right.setPower(speed);
            ccw(speed, targetAngle, corrections);
        }
    }

    public void stopMotors() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    //An attempt at making a method comparing rgb values to detect the silver
    /*public boolean checkSilver(int color) {
        if(Math.abs(Color.red(color) - Color.blue(color)) < 15 && Math.abs(Color.green(color) - Color.blue(color)) < 15 && Math.abs(Color.green(color) - Color.red(color)) < 15) {
            return true;
        } else {
            return false;
        }
    }*/

    /**
     * Does the movements based on if the gold was found in the first position
     *
     * @param program defines whether to drop our team marker in the depot or move aside (AC or AC2)
     */
    public void position1AC(int program) {
      /*  sleep(400);
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
        encoderDrives(0.5, 14, 14);
        sleep(500);
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
        proportional(0.5, 30, 2);
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

    public void markerAC() {
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
                encoderDrives(0.5, 6, 6);
                sleep(200);
                robot.Marker.setPosition(0);
                encoderDrives(0.3, -6, -6);
                proportional(0.3, 75, 2);
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
                proportional(0.5, 135, 2);
                encoderDrives(1, -69, -75);
            }
        }
    }
}

