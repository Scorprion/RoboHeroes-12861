/**
 * Created by brick on 9/9/2017.
 */
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "Test Teleop Tank Mechanum Wheels", group= "Pushbot")
public class NewTankDriveForMechanumTest extends OpMode {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public DcMotor backRight;
    public DcMotor backLeft;
    //public ColorSensor colorsensor;
    public Servo RightServo;
    public Servo LeftServo;
    public DcMotor SensorArm;
    public Servo pushyObject;
    double speed = 0;
    double turn = 0;
    double Mechaturn = 0;
    double rise = 0;
    boolean toggle = false;

    @Override
    public void init(){
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.dcMotor.get("SensorArm");
        pushyObject = hardwareMap.servo.get("pushyObject");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Say", "Press 'Start' and 'A' to begin.");
    }
    @Override
    public void init_loop(){


    }
    @Override
               public void start(){

    }
    @Override
    public void loop() {

        Mechaturn = gamepad1.left_stick_x * 1;
        speed = gamepad1.right_stick_y * 1;
        rise = gamepad1.right_stick_y * 1;
        turn = gamepad1.right_stick_x;
        if(gamepad1.dpad_up){
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);
        }
        if(gamepad1.dpad_down){
            frontLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(-speed);
        }
        if(gamepad1.dpad_right){
            frontLeft.setPower(speed - (Mechaturn*0.745));
            frontRight.setPower(speed + (Mechaturn*0.745));
            backLeft.setPower(speed + (Mechaturn*1.4));
            backRight.setPower(speed - (Mechaturn*1.4));
        }
        if(gamepad1.dpad_left){
            frontLeft.setPower(speed + 0.745);
            frontRight.setPower(speed - 0.745);
            backLeft.setPower(speed - 1.4);
            backRight.setPower(speed + 1.4);
        }
        if(gamepad1.right_stick_x >= 0.1  ){
            frontLeft.setPower(-0.9 * turn);
            frontRight.setPower(0.9 * turn);
            backLeft.setPower(-0.9 * turn);
            backRight.setPower(0.9 * turn);
        }else if (gamepad1.right_stick_x >= 0.9){
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }
        if(gamepad1.right_stick_x <= -0.1) {
            frontLeft.setPower(-0.9 * turn);
            frontRight.setPower(0.9 * turn);
            backLeft.setPower(-0.9 * turn);
            backRight.setPower(0.9 * turn);
        }else if (gamepad1.right_stick_x <= -0.9){
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
        if (gamepad1.a){
            RightServo.setPosition(0.2);
            LeftServo.setPosition(0.9);
        }
        if (gamepad1.b) {
            RightServo.setPosition(1.1);
            LeftServo.setPosition(-0.1);
        }
        if (gamepad1.right_trigger >= 0.9){
                Arm.setPower(-50);
        }
        if (gamepad1.right_bumper){
            Arm.setPower(50);
        }
        if (gamepad1.left_trigger >= 0.9){
            SensorArm.setPower(-0.1);


        }
        if (gamepad1.left_bumper){
            SensorArm.setPower(0.3);

        }
        if (gamepad1.x) {
            SensorArm.setPower(0);
        }
        if (gamepad1.y && toggle == false) {
            toggle = true;
            telemetry.addData("Toggle","is true");
        }else if (gamepad1.y && toggle == true) {
            toggle = false;
            telemetry.addData("Toggle","is false");
        }

        Arm.setPower(rise);

    }
    @Override
    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        Arm.setPower(0);
        RightServo.setPosition(0);
        LeftServo.setPosition(0.8);
    }
}



