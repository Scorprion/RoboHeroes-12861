package org.firstinspires.ftc.teamcode.Andrews_Robot.Autonomous.Init;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Andrews_Robot.Autonomous.Init.HardwareAndrews_Robot;
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
public class Andrews_RobotAggregated extends LinearOpMode {
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
    public HardwareAndrews_Robot robot = new HardwareAndrews_Robot();
    private double pidOutput = 0;

    private PID pid = new PID(0, 0, 0, 0.3);
    private ElapsedTime timer = new ElapsedTime();

    public static double color_cut = 10;

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

    public void init_vuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = robot.webcamName;

        /**
         * We also indicate which camera on the RC we wish to use.
         */

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         */
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        telemetry.addLine("Press > to start (vuforia is done)");
        telemetry.update();
    }
    public void start_vuforia() {
        targetsSkyStone.activate();

        boolean atTarget = false;
        double last_error = 0;
        while (opModeIsActive() && !atTarget) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (isD) {
                 pidDynamic(pid.likeallelse(robot.imu.getAngularOrientation().firstAngle),1.0/36.0,
                        1.22, 0.5, 0.1, 0, 0.05, null, direction.STRAFE);
                telemetry.addData("Angle: ", pid.likeallelse(robot.imu.getAngularOrientation().firstAngle));
            } else {
                robot.FrontRight.setPower(-0.05);
                robot.FrontLeft.setPower(-0.05);
                robot.BackRight.setPower(-0.05);
                robot.BackLeft.setPower(-0.05);
            }



            // Provide feedback as to where the robot is located (if we know).
            if(targetVisible) {
                translation = lastLocation.getTranslation();
                stopMotors();
                pid.setLastError(translation.get(1) / mmPerInch);  // Might not be needed?
                pid.setParams(0, 0, 0, 0.3);
                /*while(opModeIsActive() && !pid.closeEnoughTo(translation.get(1) / mmPerInch, 1, 0)) {
                    // Updating the position of the trackable skystone
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                    translation = lastLocation.getTranslation();
                    pidDynamic(translation.get(1) / mmPerInch,1/36,
                            2.5, 0.2, 0.1, 0, 0, null, direction.STRAIGHT);
                    // express position (translation) of robot in inches.
                    telemetry.addData("Error: ", last_error);
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    telemetry.update();
                }*/

                telemetry.addLine("We good");
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.update();
                // We are now at the target once the loop finishes
                atTarget = true;
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        stopMotors();
        targetsSkyStone.deactivate();
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

