package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "EncMechanumB", group = "PushBot")
public class
EncoderMechanumB extends LinearOpMode  {
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    int choice = 2;

    VuforiaLocalizer vuforia;

    HardwareMechanumDrive robot  = new HardwareMechanumDrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public NormalizedColorSensor Color_Sensor;
    public View relativeLayout;

    public boolean foundVuf = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.RightServo.setPosition(0.25);
        robot.LeftServo.setPosition(0.7);
        sleep(200);
        robot.Arm.setPower(-1);
        sleep(750);
        robot.Arm.setPower(0);
        robot.pushyObject.setPosition(0.4);
        sleep(500);
        robot.SensorArm.setPower(-0.2);
        sleep(500);
        robot.SensorArm.setPower(0);
        sleep(500);
        robot.SensorArm.setPower(-0.2);
        sleep(500);
        robot.SensorArm.setPower(0);

        cs();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double backleftInches,double backrightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() || robot.frontRight.isBusy() || robot.backLeft.isBusy() || robot.backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget,   newBackLeftTarget,   newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }


    public void cs(){

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;
        Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
        NormalizedRGBA colors = Color_Sensor.getNormalizedColors();
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
        int color = colors.toColor();
        telemetry.addLine("raw Android color: ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
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

        //lift up relic scoop thing
        robot.wtfudge.setPosition(1);
        robot.wtfudge2.setPosition(0);

        if (colors.blue > colors.red) {
            //detects Red
            robot.pushyObject.setPosition(0.2);
            sleep(100);
            robot.pushyObject.setPosition(0);
            sleep(100);
            robot.pushyObject.setPosition(0.4);
            sleep(100);
            robot.SensorArm.setPower(0.4); // raises arm
            sleep(400);
            robot.SensorArm.setPower(0);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0); // stops
        } else {

            //detects Blue
            robot.pushyObject.setPosition(0.7);
            sleep(100);
            robot.pushyObject.setPosition(1);
            sleep(100);
            robot.pushyObject.setPosition(0.4);
            sleep(100);
            robot.SensorArm.setPower(0.4); // raises arm
            sleep(400);
            robot.SensorArm.setPower(0);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0); // stops

        }
        sleep(1000);
        robot.SensorArm.setPower(0.3);
        sleep(200);
        robot.SensorArm.setPower(0);
        sleep(1000);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYc9EJX/////AAAAGTW8Yvbx50Piov359fyt2VAN3E49fsw59id8r56R5cHLGaCKa2dq45xJzb4qYBIJzeiNCHhOMIcAU8Ea8pzrinYgmKg/uBLa8m2i8t4xuYaRjVAgULijV3irq9Q82XBW+y0NtoYiInt4XVY+gxaRJfRFzoeewUw143tmuFaoBQr/8rMS3c/5sJcorWu1xFP9S+Nz7tU/1aiA0FgDYzg/utGhBqlQ85kiaEu3BALBVUKaW1Y8iv8AU6v2WvoFHpcr2K9RUDiguOU6YG5H5i/1U7mf+KhrkE+Fpz/zMVaVihDmZpCTbGEIg26BmIAXc2d2bNhSxZtK4ibOD1I2BtrM4uRW+1yX7+kPJxqjOgcPFSGk";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
        for (int i = 1; i <= 15; i++) {
            telemetry.clearAll();
            //while(!foundVuf)
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));
            telemetry.update();
            sleep(500);

            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }

            switch (vuMark) {
                case LEFT:
                    telemetry.addData("Left", "working");
                    telemetry.update();
                    foundVuf = true;
                    choice = 1;
                    break;
                case RIGHT:
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    telemetry.update();
                    foundVuf = true;
                    choice = 3;
                    break;
                case CENTER:
                    telemetry.addData("Center", "working");
                    telemetry.update();
                    foundVuf = true;
                    choice = 2;
                    break;
                default:
                    choice = 2;
            }
            if (foundVuf) {
                break;
            }
        }

            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.addData("Couldn't", "find value :/ (jk)");
            telemetry.update();

        try {
            movement();
        }
        finally{
            stop();
        }

    }

    public void movement() {

        if (choice == 1) {
            //Left
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 2.0);  // S1: Forward 12 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, 2, -2, 2, -2, 1.0);  // S3: Forward 3 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, 3, 3, 3, 3,1.0);  // S4: Reverse 3 Inches with 4 Sec timeout
            robot.RightServo.setPosition(0);
            robot.LeftServo.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 1.0);
            encoderDrive(TURN_SPEED,7,-7,7,-7,1.5);

        }
        if (choice == 2) {
            //Middle
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 2.0);  // S1: Forward 12 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, 4, -4, 4, -4, 1.0);  // S3: Forward 3 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, 4.5, 4.5, 4.5, 4.5, 1.5);  // S4: Reverse 3 Inches with 4 Sec timeout
            robot.RightServo.setPosition(0);
            robot.LeftServo.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 1.0);
            encoderDrive(TURN_SPEED,5,-5,5,-5,1.5);
        }
        if (choice == 3) {
            //Right
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 2.0);  // S1: Forward 12 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, 5.5, -5.5, 5.5, -5.5, 1.5);  // S3: Forward 3 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 1.5);  // S4: Reverse 3 Inches with 4 Sec timeout
            robot.RightServo.setPosition(0);
            robot.LeftServo.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -3, 3,-3, 3, 1.0);
            encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 1.0);
            encoderDrive(TURN_SPEED, 2, -2, 2, -2, 1.0);
            encoderDrive(DRIVE_SPEED,-1,-1,-1,-1,1.0);
            encoderDrive(TURN_SPEED,4,-4,4,-4,1.0);
        }


    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}