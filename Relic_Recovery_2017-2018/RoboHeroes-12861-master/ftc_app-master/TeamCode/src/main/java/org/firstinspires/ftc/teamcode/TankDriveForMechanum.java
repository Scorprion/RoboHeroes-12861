/**
 * Created by brick on 9/9/2017.
 */
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name= "Pushbot: Teleop Tank Mechanum Wheels", group= "Pushbot")
public class TankDriveForMechanum extends OpMode {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor Hacks; //Hacks is the relic Shovel
    //public ColorSensor colorsensor;
    public Servo RightServo;
    public Servo LeftServo;
    public DcMotor SensorArm;
    public Servo PushyObject;
    public Servo wtfudge;//wtfudge and wtfudge2 are the servos on the relic arm.
    public Servo wtfudge2;
    double speed = 0;
    double turn = 0;
    double Mechaturn = 0;
    double rise = 0;
    boolean toggle = false;

    @Override
    public void init() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        Arm = hardwareMap.dcMotor.get("Arm");
        Hacks = hardwareMap.dcMotor.get("Hacks");
        RightServo = hardwareMap.servo.get("RightServo");
        LeftServo = hardwareMap.servo.get("LeftServo");
        SensorArm = hardwareMap.dcMotor.get("SensorArm");
        PushyObject = hardwareMap.servo.get("pushyObject");
        wtfudge = hardwareMap.servo.get("wtfudge");
        wtfudge2 = hardwareMap.servo.get("wtfudge2");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Say", "Press 'Start' and 'A' to begin.");
    }
    @Override
    public void init_loop() {


    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {

        Mechaturn = gamepad1.left_stick_x * 1;
        speed = gamepad1.left_stick_y * 1;
        rise = gamepad1.right_stick_y * 1;
        turn = gamepad1.right_stick_x;

        if(gamepad1.right_stick_x >= 0.1  ){
            frontLeft.setPower(-0.9 * turn);
            frontRight.setPower(0.9 * turn);
            backLeft.setPower(-0.9 * turn);
            backRight.setPower(0.9 * turn);
        }else if (gamepad1.right_stick_x >= 0.9) {
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
        }else if (gamepad1.right_stick_x <= -0.9) {
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }
        if (gamepad1.a) {
            RightServo.setPosition(0.2);
            LeftServo.setPosition(0.9);
        }
        if (gamepad1.b) {
            RightServo.setPosition(1.1);
            LeftServo.setPosition(-0.1);
        }
        if (gamepad1.right_trigger >= 0.9) {
                Arm.setPower(-50);
        }
        if (gamepad1.right_bumper){
            Arm.setPower(50);
        }
        if (gamepad1.left_trigger >= 0.9) {
            SensorArm.setPower(-0.1);
        }
        if (gamepad1.left_bumper) {
            SensorArm.setPower(0.3);
        }
        if (gamepad1.x) {
            SensorArm.setPower(0);
        }
        if(gamepad2.left_stick_y <= -0.1 && gamepad2.left_stick_y >= -0.8 || gamepad2.right_stick_y <= -0.1 && gamepad2.right_stick_y <= -0.8){
            Hacks.setPower(0.3);
        }else if(gamepad2.left_stick_y <= -0.8 ||  gamepad2.right_stick_y <= -0.8){
            Hacks.setPower(0.4);
        }
        if(gamepad2.left_stick_y >= 0.1 && gamepad2.left_stick_y <= 0.8 || gamepad2.right_stick_y >= 0.1 && gamepad2.right_stick_y <= 0.8){
            Hacks.setPower(-0.1);
        }else if(gamepad2.left_stick_y >= 0.8  ||  gamepad2.right_stick_y >= 0.8){
            Hacks.setPower(-0.2);
        }
        if(gamepad2.left_stick_y <= -0.8 && gamepad2.right_stick_y <= -0.8){
            Hacks.setPower(0.6);
        }
        if (gamepad2.dpad_up){
            wtfudge.setPosition(1);
            wtfudge2.setPosition(0);
        }
        if (gamepad2.dpad_down){
            wtfudge.setPosition(0.3);
            wtfudge2.setPosition(0.7);
        }
        if (gamepad2.dpad_left){
            Hacks.setPower(-0.2);
        }
        if (gamepad2.dpad_right){
            Hacks.setPower(0.2);
        }
        if (gamepad2.left_bumper){
            SensorArm.setPower(0.3);
        }
        if (gamepad2.right_bumper) {
            SensorArm.setPower(0);
        }
        if (gamepad2.x){
            Hacks.setPower(0.2);
        }
        if (gamepad2.y){
            Hacks.setPower(0.1);
            wtfudge.setPosition(0.5);
        }
        if(gamepad2.b){
        wtfudge.setPosition(0.3);
        }
        if(gamepad2.a){
            frontLeft.setPower(-0.2);
            frontRight.setPower(-0.2);
            backLeft.setPower(-0.2);
            backRight.setPower(-0.2);
        }
        /*if (gamepad1.y && toggle == false) {
            toggle = true;
            telemetry.addData("Toggle","is true");
        }else if (gamepad1.y && toggle == true) {
            toggle = false;
            telemetry.addData("Toggle","is false");
        }*/
        frontLeft.setPower(speed - (Mechaturn*0.745));
        frontRight.setPower(speed + (Mechaturn*0.745));
        backLeft.setPower(speed + (Mechaturn*1.4));
        backRight.setPower(speed - (Mechaturn*1.4));
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



