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
import com.vuforia.CameraDevice;

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

@Autonomous(name="AutoTopRed", group = "PushBot")
public class AutoTopRed extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;

    public Servo RightServo;
    public Servo LeftServo;
    public Servo SensorArm;

    public int st = 0;

    public NormalizedColorSensor Color_Sensor;
    public View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.servo.get("SensorArm");
        Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        RightServo.setPosition(1.2);
        LeftServo.setPosition(-1.2);
        sleep(200);
        Arm.setPower(10);
        sleep(250);
        Arm.setPower(0);

        Thread newThread = new Thread(new Runnable() {
            public void run() {
                vuforia();
            }
        });
        newThread.start();

        waitForStart();

        SensorArm.setPosition(0.05);
        sleep(1000);
    }

        /*try {
            cs();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    public void cs() {
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        waitForStart();

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


        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        if (colors.red > colors.blue || colors.red > colors.green) {
            frontLeft.setPower(-0.1);
            frontRight.setPower(-0.1);
            sleep(1000);
            SensorArm.setPosition(1);
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            sleep(1250);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        } else {
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            sleep(1000);
            SensorArm.setPosition(1);
            frontLeft.setPower(-0.1);
            frontRight.setPower(-0.1);
            sleep(1250);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
        Stop();
    }*/

    public void vuforia() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQcFKQr/////AAAAGTK5GT2LgklXnYlg1PWBLt0Mwx3ZvLi6OILTbj0R35XhXbsKhdFrVpLG9vATT2AeMPVHdmNjDiRuH4izIk7LU7NdVsol2KUJV/V7PP/VURtd82Vb53KeS/XKYMysjASxJ3BGjtZuOnXS6KSH3errdJp3tWrE0vOGGUbDUZrqgOwg3SLfMTrOd1QhBBcdKgiupsRNhs0ylpMOYXLYzTNEZ9PGXGcj2akXjYf7iCIsGuPS4Sw5QepHqo3mGV4IPoDQ54JF3Q3fzLaxZz/UpbxoJ5Us6bCuJvXO1fn3xY/WFxiKuY6ErTR3nWh3nk/9DcQkTogf38mAzZAKmeYy1V7BfIJUWOv9sKB7HSd/x1Ne7ybz";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();

        relicTrackables.activate();

        while (true) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

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
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }


    public void left() {

    }

    public void right() {

    }

    public void center() {

    }

    public void Stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        SensorArm.setPosition(1);
        sleep(2000);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
