package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpPractice", group="Pushbot")
public class TeleOpPractice extends OpMode {
        private double robotControlSpeed = 0.7;
        private boolean used_recently = false, xbutton = false;
        private ElapsedTime timer = new ElapsedTime();

        private double turnspeed = 0;
        private double strafespeed = 0;
        private double speed = 0;

        DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Gate;
        CRServo FoundationClaw, Clamper, HeadDrop;

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

           Gate = hardwareMap.get(DcMotor.class, "Gate");

            FoundationClaw = hardwareMap.get(CRServo.class, "FoundationClaw");
            Clamper = hardwareMap.get(CRServo.class, "Clamper");
            HeadDrop = hardwareMap.get(CRServo.class, "HeadDrop");


        }

    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * robotControlSpeed;

        FrontRight.setPower(speed  - turnspeed);
        BackRight.setPower(speed - turnspeed);
        FrontLeft.setPower(speed + turnspeed);
        BackLeft.setPower(speed + turnspeed);


    }
}