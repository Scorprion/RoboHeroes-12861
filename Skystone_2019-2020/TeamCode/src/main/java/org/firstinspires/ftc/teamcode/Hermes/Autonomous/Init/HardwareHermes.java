package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@SuppressWarnings({"WeakerAccess"})
public class HardwareHermes {
    //Right motors
    public DcMotor FrontRight;
    public DcMotor BackRight;

    //Left motors
    public DcMotor FrontLeft;
    public DcMotor BackLeft;

    //The latching down boi
    public DcMotor Gate;

    //Foundation Grabber
    public CRServo FoundationClaw;
    public CRServo Clamper;

    // WebcamName webcamName = null;

    //Sensors
    public NormalizedColorSensor SkySensor1;
    public NormalizedColorSensor SkySensor2;

    //IMU sensor
    public BNO055IMU imu;

    public void init(HardwareMap ahwMap) {
        //ColorSensor = ahwMap.get(NormalizedColorSensor.class, "ColorSensor");
        //DistanceSensor = ahwMap.get(DistanceSensor.class, "DistanceSensor");

        // webcamName = ahwMap.get(WebcamName.class, "Webcam 1");


        //IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        SkySensor1 = ahwMap.get(NormalizedColorSensor.class, "SkySensor1");
        SkySensor2 = ahwMap.get(NormalizedColorSensor.class, "SkySensor2");

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

        Gate = ahwMap.get(DcMotor.class, "Gate");

        FoundationClaw = ahwMap.get(CRServo.class, "FoundationClaw");
        Clamper = ahwMap.get(CRServo.class, "Clamper");

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Make the motors not use encoders by default but you can set it to use encoders in the
        // program manually with "RUN_USING_ENCODER" and "STOP_AND_RESET_ENCODER"
    }
}
