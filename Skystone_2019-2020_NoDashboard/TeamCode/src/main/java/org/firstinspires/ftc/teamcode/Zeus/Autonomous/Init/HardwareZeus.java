package org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@SuppressWarnings({"WeakerAccess"})
public class HardwareZeus {
    //Sensors

    //Right motors
    public DcMotor FrontRight;
    public DcMotor BackRight;

    //Left motors
    public DcMotor FrontLeft;
    public DcMotor BackLeft;

    public CRServo FoundationClaw;
    CRServo CapDropper;

    public Servo StoneClampL;
    public Servo StoneClampR;

    public Servo StoneLift;
    public Servo StoneTurner;

    WebcamName webcamName = null;

    //IMU sensor
    public BNO055IMU imu;

    // The elapsed time
    public ElapsedTime runtime = new ElapsedTime();



    public void init(HardwareMap ahwMap) {
        //ColorSensor = ahwMap.get(NormalizedColorSensor.class, "ColorSensor");
        //DistanceSensor = ahwMap.get(DistanceSensor.class, "DistanceSensor");

        webcamName = ahwMap.get(WebcamName .class, "Webcam 1");

        //IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FrontRight = ahwMap.get(DcMotor.class, "FrontRight");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackRight = ahwMap.get(DcMotor.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = ahwMap.get(DcMotor.class, "FrontLeft");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackLeft = ahwMap.get(DcMotor.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Make the motors not use encoders by default but you can set it to use encoders in the
        // program manually with "RUN_USING_ENCODER" and "STOP_AND_RESET_ENCODER"

        FoundationClaw = ahwMap.get(CRServo.class, "FoundationClaw");
        StoneClampL = ahwMap.get(Servo.class, "StoneClampL");
        StoneClampR = ahwMap.get(Servo.class, "StoneClampR");

        StoneLift = ahwMap.get(Servo.class, "StoneLift");
        StoneTurner = ahwMap.get(Servo.class, "StoneTurner");

        CapDropper = ahwMap.get(CRServo.class, "CapDropper");
    }
}
