package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PID;

import java.io.File;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@SuppressWarnings({"unused", "WeakerAccess", "SameParameterValue"})
public class HermesAggregated extends LinearOpMode {
    public static final String TAG = "Vuforia Navigation Sample";

    public boolean isD = false;

    File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    public boolean LineFound = false;

    public enum direction {
        STRAFE,
        TURN,
        STRAIGHT
    }

    public enum motor_mode {
        WITHOUT_ENCODER, USING_ENCODER, STOP_RESET, RUN_TO_POSITION
    }

    public enum position {
        WALL, MIDDLE, BRIDGE, UNKNOWN
    }

    //                                              "560 rises of channel A"
    public final double countsPerInch = 54.722; // (2240 / 4?)  / (Math.PI * 2.952756)
    private ElapsedTime milliseconds = new ElapsedTime();
    public HardwareHermes robot = new HardwareHermes();
    private double pidOutput = 0;

    private PID pid = new PID(0, 0, 0, 0.3);
    private ElapsedTime timer = new ElapsedTime();

    // Vuforia variables
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AYs0VST/////AAABmSDTvCeqB0Enli9+81ZFBkxozvy8U95Nu5dzq/LLtOCRoTn0NofJMqGV9zEgfp4kCn7U9GGX+il0d08bHswAzSqj62WEOF3XBltH3XyU89HB9tTeDjH4LTVr9m6YWwqxk/z39YdfWbHt7M0mXr0lLDjGr4D1BOV4TC92aTFC7qxuH9QXgW0qRY70Zl3LO+EXsAPsNDIBUzip5UpAQ0I7Y2mi521XvWLsGnn12QnBA072X427T+A/IndZOtfRslHjHtG3Th86KBK7g8hjDCFvG//7MwhpH9XEAS6hw3o2/2Y9r1u6YTl578XAX+phQjQWvfc6t4hxdQGRts4TXWigtcD39RGH1dAHR3uLzExHpEBC";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables;

    public VectorF translation;
    public Orientation rotation;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void encoderDrives(double speed,
                              double linches,
                              double rinches,
                              double timeout // Timeout in seconds to avoid the encoders running forever
    ) {
        int newFrontRightTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newBackLeftTarget;

        if (opModeIsActive()) {
            setAllMode(motor_mode.STOP_RESET);
            setAllMode(motor_mode.USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = robot.FrontRight.getCurrentPosition() + (int) (rinches * countsPerInch);
            newBackRightTarget = robot.BackRight.getCurrentPosition() + (int) (rinches * countsPerInch);
            newFrontLeftTarget = robot.FrontLeft.getCurrentPosition() + (int) (linches * countsPerInch);
            newBackLeftTarget = robot.BackLeft.getCurrentPosition() + (int) (linches * countsPerInch);


            robot.FrontRight.setTargetPosition(newFrontRightTarget);
            robot.BackRight.setTargetPosition(newBackRightTarget);
            robot.FrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.BackLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            setAllMode(motor_mode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.BackRight.setPower(abs(speed));
            robot.FrontRight.setPower(abs(speed));
            robot.BackLeft.setPower(abs(speed));
            robot.FrontLeft.setPower(abs(speed));

            milliseconds.reset();

            while (opModeIsActive() && (milliseconds.milliseconds() < timeout * 1000) &&
                    (robot.FrontLeft.isBusy() && robot.FrontRight.isBusy() && robot.BackRight.isBusy() && robot.BackLeft.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d : %7d", (newFrontLeftTarget), (newFrontRightTarget));
                telemetry.addData("Path2", "Running at %7d : %7d : %7d : ",
                        robot.FrontRight.getCurrentPosition(),
                        robot.BackRight.getCurrentPosition(),
                        robot.FrontLeft.getCurrentPosition(),
                        robot.BackLeft.getCurrentPosition());
                telemetry.update();
            }

            stopMotors();

            // Turn off RUN_TO_POSITION and reset
            setAllMode(motor_mode.STOP_RESET);
        }
    }

    public position CheckSkySensor() {
        /**
         * This portion of program automatically calls the robot to move to check for the skystone.
         */
        boolean SkyStoneFound = false;
        int currentPos = 1;

        NormalizedColorSensor SkySensor1;
        SkySensor1 = hardwareMap.get(NormalizedColorSensor.class, "SkySensor1");
        NormalizedColorSensor SkySensor2;
        SkySensor2 = hardwareMap.get(NormalizedColorSensor.class, "SkySensor2");

        NormalizedRGBA colors = SkySensor1.getNormalizedColors();
        NormalizedRGBA colors2 = SkySensor2.getNormalizedColors();

        double max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        int color1 = colors.toColor();

        double max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);
        colors2.red /= max2;
        colors2.green /= max2;
        colors2.blue /= max2;
        int color2 = colors2.toColor();
            if (abs(Color.red(color1) - Color.red(color2)) < color_cut) {
                //Position 3
                telemetry.addLine("Position3");
                return position.BRIDGE;
            } else if (Color.red(color1) < Color.red(color2)) {
                //Position 1
                telemetry.addLine("Position1");
                return position.WALL;
            } else {
                //Position 2;
                telemetry.addLine("Position2");
                return position.MIDDLE;
            }
        }

    double out = 0;
    public void pidTurn(double P, double I, double D, double setpoint, double speed, double seconds) {
        timer.reset();
        pid.setParams(P, I, D,0.3);

        setAllMode(motor_mode.WITHOUT_ENCODER);

        // Manual error updating so h
        do {
            // Normalizes the output between 0.2 and 1 (since the robot won't even move if it's below 0.2 power)
            // out = pid.getPID(0.2 + ((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) * 0.8) / 360);
            out = pid.constrain(pid.getPID((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) / 36), -abs(speed - 1), abs(1 - speed));
            dashboardTelemetry.addData("Setpoint", setpoint);
            dashboardTelemetry.addData("Angle", pid.total_angle);
            dashboardTelemetry.addData("Delta time", pid.delta_time);
            dashboardTelemetry.addData("Delta error", pid.delta_error);
            dashboardTelemetry.addData("Error", pid.error);
            dashboardTelemetry.addData("Out", out);
            dashboardTelemetry.addData("P", pid.Poutput);
            dashboardTelemetry.addData("I", pid.Ioutput);
            dashboardTelemetry.addData("D", pid.Doutput);
            dashboardTelemetry.addData("Speed", speed + out);

            robot.FrontRight.setPower(-speed - out);
            robot.BackRight.setPower(-speed - out);
            robot.FrontLeft.setPower(speed + out);
            robot.BackLeft.setPower(speed + out);
            dashboardTelemetry.update();
        } while(opModeIsActive() && timer.milliseconds() < seconds * 1000); // && setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle) != 0);
        stopMotors();

        setAllMode(motor_mode.USING_ENCODER);
    }

    /**
     * Important Note: This method requires a manual-made loop in order to constantly pass the variable
     *
     * The dynamic version of pidTurn that should work with any variable affected by the robot's position.
     * Since Java passes by value and not by reference, it is not possible to pass the reference to
     * something like the angle, so this separate method will deal with other possibilities.
     *
     * @param variable - the manipulated/dependent variable affected by robot position changes
     * @param error_factor - scales the error by this factor (because motor powers are -1 to 1)
     * @param P - Proportional gain
     * @param I - Integral gain
     * @param D - Derivative gain
     * @param setpoint - the desired target to get the variable to
     * @param speed - the initial speed to set the motors
     * @param direc - determines how to move the robot (TURN, STRAIGHT, or STRAFE)
     *
     * @return the last PID error
     */
    public void pidDynamic(double variable, double error_factor, double P, double I, double D,
                             double setpoint, double speed, Double speed2, direction direc) {
        pid.updateParams(P, I, D);

        // In the case speed2 is given, we'll just use "speed" as the frbl
        double flbr = speed2 == null ? speed : speed2;

        // Max speed of 1
        out = pid.constrain(pid.getPID((setpoint - variable) * error_factor), -abs(speed - 1), abs(1 - speed));
        dashboardTelemetry.addData("Variable:", variable);
        dashboardTelemetry.addData("Setpint", setpoint);
        dashboardTelemetry.addData("Output", out);
        dashboardTelemetry.addData("FR", speed + out);
        dashboardTelemetry.addData("FL", flbr + out);
        dashboardTelemetry.addData("BL", speed - out);
        dashboardTelemetry.addData("BR", flbr + out);
        dashboardTelemetry.update();

        if(direc == direction.STRAFE) {
            robot.FrontRight.setPower(speed - out);
            robot.BackRight.setPower(flbr - out);
            robot.FrontLeft.setPower(flbr + out);
            robot.BackLeft.setPower(speed + out);
        } else if(direc == direction.STRAIGHT){
            robot.FrontRight.setPower(speed + out);
            robot.BackRight.setPower(flbr + out);
            robot.FrontLeft.setPower(flbr + out);
            robot.BackLeft.setPower(speed + out);
        } else if(direc == direction.TURN){
            robot.FrontRight.setPower(-speed - out);
            robot.BackRight.setPower(speed + out);
            robot.FrontLeft.setPower(speed + out);
            robot.BackLeft.setPower(-speed - out);
        }

        dashboardTelemetry.addData("P", pid.Poutput);
        dashboardTelemetry.addData("I", pid.Ioutput);
        dashboardTelemetry.addData("D", pid.Doutput);
        dashboardTelemetry.addData("Setpoint", setpoint);
        dashboardTelemetry.addData("Variable", variable);
        dashboardTelemetry.addData("Error", pid.error);
        dashboardTelemetry.addData("Last error", pid.lasterror);
        dashboardTelemetry.addData("Actual error", (setpoint - variable) * error_factor);
        dashboardTelemetry.addData("Out", out);
        dashboardTelemetry.update();
    }

    public void mecanumMove(double speed, double angle, double inches, double timer) {
        // Turn off RUN_TO_POSITION and reset
        setAllMode(motor_mode.STOP_RESET);
        setAllMode(motor_mode.USING_ENCODER);

        /*double current_angle = robot.imu.getAngularOrientation().firstAngle >= 0 ?
                robot.imu.getAngularOrientation().firstAngle : robot.imu.getAngularOrientation().firstAngle + 360;
        double radians = Math.toRadians(current_angle - angle);*/
        double radians = Math.toRadians(angle);
        double setpoint = likeallelse(robot.imu.getAngularOrientation().firstAngle);  // We want to remain at the same angle we started
        double last_error = 0;
        double forward_percent = Math.cos(radians),
                sideways_percent = Math.sin(radians);
        double flbr, frbl;
        int distanceflbr, distancefrbl;

        flbr = round(speed * (forward_percent + sideways_percent), 2); // speed * forward_percent + speed * sideways_percent
        frbl = round(speed * (forward_percent - sideways_percent), 2);

        distanceflbr = (int)abs(countsPerInch * inches);
        distancefrbl = (int)abs(countsPerInch * inches);

        robot.FrontRight.setPower(frbl);
        robot.BackRight.setPower(flbr);
        robot.FrontLeft.setPower(flbr);
        robot.BackLeft.setPower(frbl);

        // pid.setLastError(0); Might or might not be needed
        milliseconds.reset();

        // Warning - Will be inconsistent, but will works
        while (opModeIsActive() && (milliseconds.milliseconds() < timer * 1000)
                && (abs(robot.FrontRight.getCurrentPosition()) < distancefrbl
                    || abs(robot.FrontLeft.getCurrentPosition()) < distanceflbr)) {
            // We're gonna try PID yo
            pidDynamic(likeallelse(robot.imu.getAngularOrientation().firstAngle),1.0 / 36.0,
                     1.0, 0.25, 0.04, setpoint, frbl, flbr, direction.STRAFE);
            telemetry.addData("Angle: ", pid.likeallelse(robot.imu.getAngularOrientation().firstAngle));

            // Display it for the driver.
            telemetry.addLine()
                    .addData("Forward: ", forward_percent)
                    .addData("Sideways: ", sideways_percent);
            telemetry.addData("FLBR Speed: ", flbr);
            telemetry.addData("FRBL Speed: ", frbl);
            telemetry.addData("Path1", "Running to %7d and %7d", distanceflbr, distancefrbl);
            telemetry.addData("Path2", "Running at %7d : %7d : %7d : %7d",
                    robot.FrontRight.getCurrentPosition(),
                    robot.BackRight.getCurrentPosition(),
                    robot.FrontLeft.getCurrentPosition(),
                    robot.BackLeft.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void stopMotors() {
        robot.FrontRight.setPower(0);
        robot.BackRight.setPower(0);
        robot.FrontLeft.setPower(0);
        robot.BackLeft.setPower(0);
    }
    public void setAllMode(motor_mode mode) {
        if(mode == motor_mode.RUN_TO_POSITION) {
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(mode == motor_mode.STOP_RESET) {
            robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if(mode == motor_mode.USING_ENCODER) {
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(mode == motor_mode.WITHOUT_ENCODER) {
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_DOWN);
        return bd.doubleValue();
    }
    public boolean inrange(double value, double min, double max) {
        return value > min && value < max;
    }
    public boolean inrange(double value, double minmax) {
        return value > -minmax && value < minmax;
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
        return((error == value)? true: false);*/
        return round(round_value, place) == value;
    }
    public double likeallelse(double angle) {
        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;

        return angle;
    }
}

