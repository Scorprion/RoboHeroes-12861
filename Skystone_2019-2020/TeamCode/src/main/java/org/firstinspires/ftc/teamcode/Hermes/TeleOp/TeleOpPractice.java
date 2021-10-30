package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpPractice", group="Pushbot")
public class TeleOpPractice extends OpMode {
        private double robotControlSpeed = 0.7;
        private boolean used_recently = false, xbutton = false;
        private ElapsedTime timer = new ElapsedTime();

        private double turnspeed = 0;
        private double strafespeed = 0;
        private double speed = 0;

        private boolean baseClaw = false;
        private boolean Clamp = false;

        DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Gate;
        Servo FoundationClaw, Clamper, HeadDrop;

        public void init(){
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

            //Gate = hardwareMap.get(DcMotor.class, "Gate");

            FoundationClaw = hardwareMap.get(Servo.class, "Arm");
            Clamper = hardwareMap.get(Servo.class, "Pincher");
            //HeadDrop = hardwareMap.get(CRServo.class, "HeadDrop");


        }

    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * robotControlSpeed;

        FrontRight.setPower(speed  - turnspeed);
        BackRight.setPower(speed - turnspeed);
        FrontLeft.setPower(speed + turnspeed);
        BackLeft.setPower(speed + turnspeed);

        if(gamepad2.x && baseClaw){
            baseClaw = false;
            used_recently = true;
            timer.reset();

        }else if(gamepad2.x){
            baseClaw = true;
            used_recently = true;
            timer.reset();
        }

        if(gamepad2.y && Clamp){
            Clamp = false;
            used_recently = true;
            timer.reset();
        }else if(gamepad2.y){
            Clamp = true;
            used_recently = true;
            timer.reset();
        }

        if(used_recently) {
            if(timer.milliseconds() > 1500) {
                used_recently = false;
            }
        }

        if(baseClaw) {
            FoundationClaw.setPosition(1);
        }else{
            FoundationClaw.setPosition(0);
        }

        if(Clamp) {
            Clamper.setPosition(0);
        }else{
            Clamper.setPosition(1);
        }
    }

}