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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "MechanumClean", group = "PushBot")
public class MechanumClean extends LinearOpMode {

    //public static final String TAG = "Vuforia VuMark Sample";

    //OpenGLMatrix lastLocation = null;

     public VuforiaLocalizer vuforia;

    public DcMotor frontRight; // frontRight motor
    public DcMotor frontLeft;  // frontLeft motor
    public DcMotor Arm; // Arm motor
    public DcMotor backRight; // backRight motor
    public DcMotor backLeft; // backLeft motor
    public Servo RightServo; // RightServo
    public Servo LeftServo; // LeftServo
    public Servo SensorArm; // SensorArm servo
    public float[] hsvValues = new float[3];
    public final float values[] = hsvValues;

    public NormalizedColorSensor Color_Sensor;
    public View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.servo.get("SensorArm"); // Initializing everything in the hardwareMap
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE); // reversing direction to make controlling them less confusing

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId); //

        waitForStart(); // Initializing (and not moving) before "play" pressed

        RightServo.setPosition(0);
        LeftServo.setPosition(0.8); // opens servos
        sleep(200);
        Arm.setPower(12); // raises clamp
        sleep(750);
        Arm.setPower(0); // stops clamp
        SensorArm.setPosition(0.35);
        sleep(1000);
        SensorArm.setPosition(0.5);
        sleep(1000); // sets down arm (slow enough to not damage it)

        try {
            cs();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        } // makes a thread to refer to the "cs" method while still running this method in the background

    }

    public void cs() {
        NormalizedRGBA colors = Color_Sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues); // get values and convert to HSV (Hue, Saturation, Value)

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        int color = colors.toColor();
        color = colors.toColor();

        Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        if (colors.red > colors.blue || colors.red > colors.green) {
            //detects Red
            frontLeft.setPower(0.115);
            frontRight.setPower(0.115);
            backLeft.setPower(0.115);
            backRight.setPower(0.115); // moves forward
            sleep(1500);
            sleep(500);
            SensorArm.setPosition(0); // raises arm
            frontLeft.setPower(-0.125);
            frontRight.setPower(-0.125);
            backLeft.setPower(-0.125);
            backRight.setPower(-0.125); // moves backward
            sleep(1500);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0); // stops
        } else {
            //detects Blue
            frontLeft.setPower(-0.125);
            frontRight.setPower(-0.125);
            backLeft.setPower(-0.125);
            backRight.setPower(-0.125); // moves backward
            sleep(1000);
            sleep(500);
            SensorArm.setPosition(0); // raises arm
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            backLeft.setPower(0.1);
            backRight.setPower(0.1); // moves forward (slowly)
            sleep(1000);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0); // stops

        }
        sleep(2000);
        SensorArm.setPosition(0);
        vuScanner();
    }

    public void vuScanner() {

    }

    //added so you won't get a NullPointerException error
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
