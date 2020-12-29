package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@SuppressWarnings({"WeakerAccess"})
public class HardwareHermes {
    //Right motors
    public DcMotorEx FrontRight;
    public DcMotorEx BackRight;

    //Left motors
    public DcMotorEx FrontLeft;
    public DcMotorEx BackLeft;

    //The latching down boi
    public DcMotorEx Gate;

    //Foundation Grabber
    public CRServo FoundationClaw;
    public CRServo Clamper;

    // Distance Sensor
    public DistanceSensor ds;

    // Color Sensors (if you cant read)
    public ColorSensor ringcs;

    //IMU sensor
    public BNO055IMU imu;

    // The elapsed time
    public ElapsedTime runtime = new ElapsedTime();



    public void init(HardwareMap ahwMap) {
        ringcs = ahwMap.get(ColorSensor.class, "RingColorSensor");
        ds = ahwMap.get(DistanceSensor.class, "DistanceSensor");

        //IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = false;
        // parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator(0.01, 0.2);
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FrontRight = ahwMap.get(DcMotorEx.class, "FrontRight");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackRight = ahwMap.get(DcMotorEx.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = ahwMap.get(DcMotorEx.class, "FrontLeft");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackLeft = ahwMap.get(DcMotorEx.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gate = ahwMap.get(DcMotorEx.class, "Gate");

        FoundationClaw = ahwMap.get(CRServo.class, "FoundationClaw");
        Clamper = ahwMap.get(CRServo.class, "Clamper");

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
    }
}
