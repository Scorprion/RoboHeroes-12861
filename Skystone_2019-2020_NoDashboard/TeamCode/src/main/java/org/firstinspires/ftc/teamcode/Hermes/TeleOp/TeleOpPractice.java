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

    DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Gate, Spinner;
    CRServo Clamper;

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
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");

        Clamper = hardwareMap.get(CRServo.class, "Clamper");


    }

    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * robotControlSpeed;

        FrontRight.setPower(speed + strafespeed - turnspeed);
        BackRight.setPower(speed - strafespeed - turnspeed);
        FrontLeft.setPower(speed - strafespeed + turnspeed);
        BackLeft.setPower(speed + strafespeed + turnspeed);

        if (gamepad1.a) {
            Clamper.setPower(-0.7);
        } else {
            Clamper.setPower(0.7);
        }

        if ((gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1)) {
            Gate.setPower(gamepad1.left_stick_y * 0.4);
        }else{
            Gate.setPower(0);
        }

        if (gamepad1.y) {
            Spinner.setPower(0.7);
        }else{
            Spinner.setPower(0);
        }
    }
}