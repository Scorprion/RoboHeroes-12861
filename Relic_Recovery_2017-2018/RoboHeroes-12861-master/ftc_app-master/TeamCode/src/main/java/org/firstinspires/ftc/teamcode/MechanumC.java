package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
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

/**
 * Created by brick on 12/9/2017.
 */

@Autonomous(name = "MechanumC", group = "PushBot")
public class MechanumC extends LinearOpMode {


    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor SensorArm;
    //public ColorSensor colorsensor;
    public Servo RightServo;
    public Servo LeftServo;
    public Servo pushyObject;
    double speed = 0;
    double turn = 0;
    double arm = 0;
    double rise = 0;
    public NormalizedColorSensor Color_Sensor;
    public View relativeLayout;

    @Override

    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.dcMotor.get("SensorArm");
        pushyObject = hardwareMap.servo.get("pushyObject");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        RightServo.setPosition(0.25);
        LeftServo.setPosition(0.7);
        sleep(200);
        Arm.setPower(-1);
        sleep(750);
        Arm.setPower(0);
        pushyObject.setPosition(0.4);
        sleep(500);
        SensorArm.setPower(-0.2);
        sleep(500);
        SensorArm.setPower(0);
        sleep(500);
        SensorArm.setPower(-0.2);
        sleep(500);
        SensorArm.setPower(0);


        try {
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
        if (colors.blue > colors.red) {
            //detects Blue
            pushyObject.setPosition(0.7);
            sleep(100);
            pushyObject.setPosition(1);
            sleep(200);
            pushyObject.setPosition(0.4);
            sleep(100);
            SensorArm.setPower(0.4); // raises arm
            sleep(350);
            SensorArm.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0); // stops
        } else {
            //detects Red
            pushyObject.setPosition(0.2);
            sleep(100);
            pushyObject.setPosition(0);
            sleep(200);
            pushyObject.setPosition(0.4);
            sleep(100);
            SensorArm.setPower(0.4); // raises arm
            sleep(350);
            SensorArm.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0); // stops

        }
        sleep(2000);
        SensorArm.setPower(0.2);
        sleep(700);
        SensorArm.setPower(0);
        sleep(2000);


            sleep(1000);




            //TODO fix this please
            telemetry.addData("VuMark", "not visible");
            frontLeft.setPower(-0.3);
            frontRight.setPower(-0.3);
            backLeft.setPower(-0.3);
            backRight.setPower(-0.3);
            sleep(1800);
            frontLeft.setPower(0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(0.4);
            backRight.setPower(-0.4);
            sleep(1750);
            /*frontLeft.setPower(-0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(-0.4);
            backRight.setPower(-0.4);
            sleep(100);*/
            frontLeft.setPower(0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(0.4);
            backRight.setPower(-0.4);
            sleep(750);


            frontLeft.setPower(0.2);
            frontRight.setPower(0.2);
            backLeft.setPower(0.2);
            backRight.setPower(0.2);
            sleep(850);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sleep(250);
            RightServo.setPosition(1);
            LeftServo.setPosition(0);
            sleep(500);
            frontLeft.setPower(-0.15);
            frontRight.setPower(-0.15);
            backLeft.setPower(-0.15);
            backRight.setPower(-0.15);
            sleep(400);
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);




    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
