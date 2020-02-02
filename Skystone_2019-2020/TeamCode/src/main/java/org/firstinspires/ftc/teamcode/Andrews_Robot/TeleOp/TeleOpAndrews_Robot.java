package org.firstinspires.ftc.teamcode.Andrews_Robot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "TeleOp Andrews_Robot", group= "Pushbot")
public class TeleOpAndrews_Robot extends OpMode {
    private double robotControlSpeed = 0.7;
    private boolean used_recently = false, xbutton = false;
    private ElapsedTime timer = new ElapsedTime();

    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;

    DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Gate;
    CRServo FoundationClaw, Clamper, Arm;
    Servo Release;

    public void init() {
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Gate = hardwareMap.get(DcMotor.class, "Gate");

        FoundationClaw = hardwareMap.get(CRServo.class, "FoundationClaw");
        Clamper = hardwareMap.get(CRServo.class, "Clamper");

        Arm = hardwareMap.get(CRServo.class, "Arm");
        Release = hardwareMap.get(Servo.class, "Release");

    }

    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 1         |
        |                           |
        |                           |
        -----------------------------
         */

        if (gamepad2.a) {
            Clamper.setPower(-0.7);
        } else {
            Clamper.setPower(0.7);
        }

        if ((gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1)) {
            Gate.setPower(gamepad2.left_stick_y * 0.4);
        }else{
            Gate.setPower(0);
        }

        if(gamepad2.right_stick_y >= 0.1){
            FoundationClaw.setPower(-1);
        }else if(gamepad2.right_stick_y <= -0.1){
            FoundationClaw.setPower(1);
        }else{
            FoundationClaw.setPower(0);
        }

        if(gamepad2.x){
            Release.setPosition(0.4);
//            timer.reset();
//            xbutton = true;
//        }else if(xbutton) {
//            // The x button is released
//            used_recently = true;
//            xbutton = false;
//        }else if(used_recently) {
//            Release.setPosition(0.2);
//            if(timer.milliseconds() > 1500) {
//                used_recently = false;
//            }
        }else{
            // Default, don't move
            Release.setPosition(0);
        }

        if (gamepad2.y){

                Arm.setPower(-1);


        }else{
            Arm.setPower(0);
        }

        //Strafing
        FrontRight.setPower(speed - strafespeed - turnspeed);
        BackRight.setPower(speed + strafespeed - turnspeed);
        FrontLeft.setPower(speed + strafespeed + turnspeed);
        BackLeft.setPower(speed - strafespeed + turnspeed);

        telemetry.addData("Speeds: ", "%.5f, %.5f, %.5f", (speed), (strafespeed), (turnspeed));
        telemetry.update();
    }

}
