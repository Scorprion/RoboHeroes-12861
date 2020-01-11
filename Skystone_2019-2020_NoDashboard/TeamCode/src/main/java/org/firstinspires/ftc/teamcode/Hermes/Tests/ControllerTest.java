package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "Controller Test", group= "Pushbot")
public class ControllerTest extends OpMode {
    double strafespeed = 0, speed = 0;

    DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Gate;
    CRServo FoundationClaw, HeadDrop;

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

        HeadDrop = hardwareMap.get(CRServo.class, "HeadDrop");
    }

    @Override
    public void loop() {
        strafespeed = gamepad1.left_stick_x * 1;
        speed = gamepad1.left_stick_y * -1;

        telemetry.addData("Speeds: ", "%.5f, %.5f", (speed), (strafespeed));
        telemetry.addData("Sum: ", speed + strafespeed);
        telemetry.addData("Diff: ", speed - strafespeed);
        telemetry.update();
    }
}
