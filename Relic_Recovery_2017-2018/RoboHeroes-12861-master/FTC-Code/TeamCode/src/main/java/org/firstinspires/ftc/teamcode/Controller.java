package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "TeleOp", group= "Pushbot")
public class Controller extends OpMode {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public Servo RightServo;
    public Servo LeftServo;
    public Servo SensorArm;
    double speed = 0;
    double turn = 0;
    double arm = 0;
    double rise = 0;

    @Override
    public void init(){
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.servo.get("SensorArm");
        //Colorsensor = hardwareMap.colorSensor.get("ColorSensor");
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Say", "Press 'Start' and 'A' to begin.");
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start() {
        SensorArm.setPosition(1);
    }
    @Override
    public void loop() {


        turn = gamepad1.left_stick_x;
        speed = gamepad1.left_stick_y * -1.5;
        rise = gamepad1.right_stick_y * -1;


        if (gamepad1.y) {

        }
        if (gamepad1.x) {

        }
        if (gamepad1.a){
            RightServo.setPosition(0.4);
            LeftServo.setPosition(0.4);
        }
        if (gamepad1.b) {
            RightServo.setPosition(1);
            LeftServo.setPosition(-1.2);
        }
        if (gamepad1.x) {
            frontLeft.setPower(300);
            frontRight.setPower(300);
        }
        if (gamepad1.y) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
        frontLeft.setPower(speed + turn);
        frontRight.setPower(speed - turn);
        Arm.setPower(rise);
    }
    @Override
    public void stop(){

    }
}

