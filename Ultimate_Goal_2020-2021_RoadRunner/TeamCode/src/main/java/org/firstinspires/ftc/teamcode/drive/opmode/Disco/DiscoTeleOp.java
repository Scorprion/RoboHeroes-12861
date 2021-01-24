package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "DiscoTeleOp", group= "Disco")
public class DiscoTeleOp extends OpMode{

    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;
    DcMotor ShooterL, ShooterR;
    DcMotor Intake, WobbleSet;

    Servo WobbleClipR,Twitching;

    Servo WobbleGrab;

    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;
    private double robotControlSpeed = 1;
    private double shootingControlSpeed = 0.8;

    private double CRconfigZero = 0.0;

    private boolean SpeedToggle = true;
    private boolean usedRecently = false;

    public ElapsedTime runtime = new ElapsedTime();

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

        //Shooter Motors
        ///*
        ShooterL = hardwareMap.get(DcMotor.class, "ShooterL");
        ShooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ShooterR = hardwareMap.get(DcMotor.class, "ShooterR");
        ShooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        WobbleSet = hardwareMap.get(DcMotor.class, "WobbleSet");
        WobbleSet.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //WobbleClipR = hardwareMap.get(Servo.class, "WobbleClipR");

        //Twitching = hardwareMap.get(Servo.class, "Twitching");

        WobbleGrab = hardwareMap.get(Servo.class, "WobbleGrab");

        //*/
        //CRconfigZero = WobbleGrab.getPower();
    }

    public void loop(){

        //movement
        turnspeed = gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        if(gamepad1.right_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0|| gamepad1.left_stick_y != 0){
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

        //if(gamepad1.right_stick_y != 0) {
        if(gamepad1.b){
            Intake.setPower(-1);
        }else if(gamepad1.a) {
            Intake.setPower(1);
        }else{
            Intake.setPower(0);
        }

        /*if(gamepad1.left_bumper){
            Twitching.setPosition(-1);
            if(Twitching.getPosition()<=0){
                Twitching.setPosition(1);
            }else{
                Twitching.setPosition(0);
            }
        }*/

        //Hardcoded Shooting Speeds

        if(gamepad2.a){

            //Testing
            /*
            FrontRight.setPower(0.8);
            BackRight.setPower(0.8);
            */
            shootingControlSpeed = 0.5;

        }else if(gamepad2.x){

            //Testing
            /*
            FrontRight.setPower(0.8);
            BackRight.setPower(0.8);
            */
            shootingControlSpeed = 0.475;

            //}else if(gamepad2.y){

            //Testing
            /*
            FrontRight.setPower(0.8);
            BackRight.setPower(0.8);
            */
            //shootingControlSpeed = 0.5;

        }else{
            shootingControlSpeed = 0;
        }

        //Intake and transport mechanism

        if(gamepad2.right_stick_y != 0){
            ShooterL.setPower(gamepad2.right_stick_y);
            ShooterR.setPower(gamepad2.right_stick_y);
        }else if(gamepad2.a||gamepad2.b||gamepad2.x||gamepad2.y){
            ShooterL.setPower(shootingControlSpeed);
            ShooterR.setPower(shootingControlSpeed);
        }else{
            ShooterL.setPower(0);
            ShooterR.setPower(0);
        }

        //Wobble Goal Hand

        if(gamepad2.left_bumper){
            WobbleGrab.setPosition(1);
        }else if(gamepad2.right_bumper){
            WobbleGrab.setPosition(0);
        }else if(gamepad2.right_trigger>0){
            WobbleGrab.setPosition(0.5);
        }

        //Wobble Goal Clip NONFUNCTIONAL

        /*if(gamepad2.left_trigger > 0){
            WobbleClipR.setPosition(0.9);
        }else{
            WobbleClipR.setPosition(1);
        }*/

        //Wobble goal main Arm

        if(gamepad2.left_stick_y != 0){
            WobbleSet.setPower(gamepad2.left_stick_y);
        }else{
            WobbleSet.setPower(0);
        }

        /*if(gamepad2.b && !usedRecently){
            if(shootingControlSpeed >= 1.0) {
                shootingControlSpeed = 0;
            }else{
                shootingControlSpeed += 0.2;
            }
            runtime.reset();
            usedRecently = true;
        }


        if (runtime.milliseconds() > 200) {
            usedRecently = false;
        }*/

        telemetry.addData("Speeds: ", "%.5f, %.5f, %.5f", (speed), (strafespeed), (turnspeed));
        telemetry.addData("Shooter Speed:",gamepad2.right_stick_y);
        telemetry.addData("HardSpeeds:","y: %.5f, x: %.5f, a: %.5f",(0.5),(0.6),(0.7));
        telemetry.addData("Value: ",gamepad2.left_trigger);
        telemetry.addData("Value: ",WobbleGrab);
        telemetry.update();
    }
}
