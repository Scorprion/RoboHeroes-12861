package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMechanumDrive {

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor SensorArm;
    public DcMotor Hacks;
    public NormalizedColorSensor Color_Sensor;
    public Servo RightServo;
    public Servo LeftServo;
    public Servo pushyObject;
    public Servo wtfudge;
    public Servo wtfudge2;
    //public ColorSensor colorsensor;
    double speed = 0;
    double turn = 0;
    double arm = 0;
    double rise = 0;
    public View relativeLayout;
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        Arm    = hwMap.get(DcMotor.class, "Arm");
        SensorArm = hwMap.get(DcMotor.class, "SensorArm");
        Color_Sensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        Arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        LeftServo  = hwMap.get(Servo.class, "LeftServo");
        RightServo = hwMap.get(Servo.class, "RightServo");
        pushyObject = hwMap.get(Servo.class, "pushyObject");
        wtfudge = hwMap.get(Servo.class, "wtfudge");
        wtfudge2 = hwMap.get(Servo.class, "wtfudge2");
    }
}
