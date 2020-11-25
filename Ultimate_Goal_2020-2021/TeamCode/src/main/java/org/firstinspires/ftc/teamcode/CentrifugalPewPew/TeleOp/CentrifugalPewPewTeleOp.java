package org.firstinspires.ftc.teamcode.CentrifugalPewPew.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "TeleOp", group= "Controlled")
public class CentrifugalPewPewTeleOp extends OpMode {

    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;

    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;
    private double robotControlSpeed = 1;


    public void init(){
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){
        turnspeed = gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        if(gamepad1.right_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0|| gamepad1.left_stick_x != 0){
            FrontRight.setPower(speed - strafespeed - turnspeed);
            BackRight.setPower(speed + strafespeed - turnspeed);
            FrontLeft.setPower(speed + strafespeed + turnspeed);
            BackLeft.setPower(speed - strafespeed + turnspeed);
        }else{
            FrontRight.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
        }
    }
}
