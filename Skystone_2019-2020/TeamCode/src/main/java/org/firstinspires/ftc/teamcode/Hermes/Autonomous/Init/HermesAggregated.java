package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PID;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

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

    public final double countsPerInch = 54.722;
    private ElapsedTime milliseconds = new ElapsedTime();
    public HardwareHermes robot = new HardwareHermes();
    private double pidOutput = 0;

    private PID pid = new PID(0, 0, 0, 0);
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

    @Override
    public void runOpMode() throws InterruptedException {
        // Does nothing here
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
            // Determine new target position, and pass to motor controller
            newFrontRightTarget = robot.FrontRight.getCurrentPosition() + (int)(rinches * countsPerInch);
            newBackRightTarget = robot.BackRight.getCurrentPosition() + (int)(rinches * countsPerInch);

            newFrontLeftTarget = robot.FrontLeft.getCurrentPosition() + (int)(linches * countsPerInch);
            newBackLeftTarget = robot.BackLeft.getCurrentPosition() + (int)(linches * countsPerInch);


            robot.FrontRight.setTargetPosition(newFrontRightTarget);
            robot.BackRight.setTargetPosition(newBackRightTarget);
            robot.FrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.BackLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            robot.BackRight.setPower(Math.abs(speed));
            robot.FrontRight.setPower(Math.abs(speed));

            robot.BackLeft.setPower(Math.abs(speed));
            robot.FrontLeft.setPower(Math.abs(speed));

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
            robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stopMotors() {
        robot.FrontRight.setPower(0);
        robot.BackRight.setPower(0);
        robot.FrontLeft.setPower(0);
        robot.BackLeft.setPower(0);
    }

    double out = 0;
    public void pidTurn(double P, double I, double D, double setpoint, double speed, double seconds) {
        timer.reset();
        pid.setParams(P, I, D, null);
        pid.update_error((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) / 10);

        while(opModeIsActive() && timer.milliseconds() < seconds * 1000 && pid.error != 0) {
            telemetry.addLine()
                    .addData("P", P)
                    .addData("I", I)
                    .addData("D", D);
            out = pid.getPID((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) / 10);
            telemetry.addLine()
                    .addData("Angle", pid.total_angle)
                    .addData("Out", out)
                    .addData("Speed", speed + out);
            robot.FrontRight.setPower(-speed - out);
            robot.BackRight.setPower(-speed - out);
            robot.FrontLeft.setPower(speed + out);
            robot.BackLeft.setPower(speed + out);
            telemetry.update();
        }
        stopMotors();
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
     * @param turn - whether or not to turn (motor powers are adjusted)
     *
     * @return the last PID error
     */
    public double pidDynamic(double variable, double lasterror, double error_factor, double P, double I, double D,
                             double setpoint, double speed, boolean turn) {
        pid.setParams(P, I, D, lasterror);
        // pid.update_error(variable - setpoint);
        out = pid.getPID((setpoint - variable) * error_factor);
        if(turn) {
            robot.FrontRight.setPower(speed + out);
            robot.BackRight.setPower(speed - out);
            robot.FrontLeft.setPower(speed - out);
            robot.BackLeft.setPower(speed + out);
        } else {
            robot.FrontRight.setPower(speed + out);
            robot.BackRight.setPower(speed + out);
            robot.FrontLeft.setPower(speed + out);
            robot.BackLeft.setPower(speed + out);
        }
        return pid.error;
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
                last_error = pidDynamic((pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)), last_error,1/10,
                        1.5, 0.5, 0, 0, 0.05, true);
                telemetry.addData("Angle: ", pid.likeallelse(robot.imu.getAngularOrientation().firstAngle));
            } else {
                robot.FrontRight.setPower(-0.057);
                robot.FrontLeft.setPower(-0.05);
                robot.BackRight.setPower(-0.057);
                robot.BackLeft.setPower(-0.05);
            }



            // Provide feedback as to where the robot is located (if we know).
            if(targetVisible) {
                translation = lastLocation.getTranslation();
                pid.setParams(0, 0, 0, 0.0);
                stopMotors();
                last_error = 0;
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
                    last_error = pidDynamic(translation.get(1) / mmPerInch, last_error, 0.01,
                            1.5, 0.2, 0, 0, 0, false);
                    // express position (translation) of robot in inches.
                    translation = lastLocation.getTranslation();
                    telemetry.addData("Error: ", last_error);
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    telemetry.update();
                } */

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


    public void LineReading() {

        /** The colorSensor field will contain a reference to our color sensor hardware object */
        NormalizedColorSensor colorSensor;
        /** The relativeLayout field is used to aid in providing interesting visual feedback
         * in this sample application; you probably *don't* need something analogous when you
         * use a color sensor on your robot */
        View relativeLayout;

        /**
         * The runOpMode() method is the root of this LinearOpMode, as it is in all linear opModes.
         * Our implementation here, though is a bit unusual: we've decided to put all the actual work
         * in the main() method rather than directly in runOpMode() itself. The reason we do that is that
         * in this sample we're changing the background color of the robot controller screen as the
         * opmode runs, and we want to be able to *guarantee* that we restore it to something reasonable
         * and palatable when the opMode ends. The simplest way to do that is to use a try...finally
         * block around the main, core logic, and an easy way to make that all clear was to separate
         * the former from the latter in separate methods.
         */

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // bPrevState and bCurrState keep track of the previous and current state of the button
        boolean bPrevState = false;
        boolean bCurrState = false;

        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


        // Loop until we are asked to stop
        while (opModeIsActive() && !LineFound) {
            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            //Reads the Red Line and drops the SkyStone
            if ((colors.red) < 60 && (colors.red > 100) || (colors.blue < 70) && (colors.blue > 100)) {
                stopMotors();
                encoderDrives(0.5, 17, 17, 5);
                robot.Gate.setPower(0.5);
                encoderDrives(0.5, -6, -6, 3);
                LineFound = true;
            }

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            telemetry.update();

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
        }
    }


    public void mecanumMove(double speed, double angle, double inches, double timer) {
        robot.FrontRight.setPower(-0.2);
        robot.BackRight.setPower(-0.2);
        robot.FrontLeft.setPower(-0.2);
        robot.BackLeft.setPower(-0.2);
        sleep(500);
        // Turn off RUN_TO_POSITION and reset
        robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*double current_angle = robot.imu.getAngularOrientation().firstAngle >= 0 ?
                robot.imu.getAngularOrientation().firstAngle : robot.imu.getAngularOrientation().firstAngle + 360;
        double radians = Math.toRadians(current_angle - angle);*/
        double radians = Math.toRadians(angle);
        double forward_percent = Math.cos(radians),
                sideways_percent = Math.sin(radians);
        double flbr, frbl;
        int distanceflbr, distancefrbl;

        flbr = round(speed * (forward_percent + sideways_percent), 2); // speed * forward_percent + speed * sideways_percent
        frbl = round(speed * (forward_percent - sideways_percent), 2);

        distanceflbr = (int)((countsPerInch * inches) * (forward_percent + sideways_percent));
        distancefrbl = (int)((countsPerInch * inches) * (forward_percent - sideways_percent));

        robot.FrontRight.setPower(frbl);
        robot.BackRight.setPower(flbr);
        robot.FrontLeft.setPower(flbr);
        robot.BackLeft.setPower(frbl);

        milliseconds.reset();

        // Warning - Will be inconsistent, but it works
        while (opModeIsActive() && (milliseconds.milliseconds() < timer * 1000)
                && (inrange(robot.FrontRight.getCurrentPosition(), -distancefrbl, distancefrbl)
                    || inrange(robot.FrontLeft.getCurrentPosition(), -distanceflbr, distanceflbr))) {
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

    public double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_DOWN);
        return bd.doubleValue();
    }

    public boolean inrange(double value, double min, double max) {
        return value >= min && value <= max;
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

