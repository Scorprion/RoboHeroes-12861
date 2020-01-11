package org.firstinspires.ftc.teamcode.Zeus.TeleOp;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@SuppressWarnings({"unused", "WeakerAccess", "SameParameterValue", "FieldCanBeLocal"})
@TeleOp(name= "Teleop Zeus", group= "Pushbot")
public class TeleOpZeus extends OpMode {

        //Making the slower arm and elbow toggle (driver 2)
        private ElapsedTime openClaw = new ElapsedTime();
        private boolean switchedS = false;
        private boolean usedRecently = false;
        private double controlSpeedE = 1, controlSpeedS = 1;

        //Making the slower robot toggle (driver 1)
        private ElapsedTime rmove = new ElapsedTime();

        private boolean robotCSpeed = false; // the robot's speed to be able to slow it down
        private boolean robotUsedRecent = false;
        private boolean StopperTouched = false;
        private double robotControlSpeed = 1;

        private double turnspeed = 0;
        private double strafespeed = 0;
        private double speed = 0;

        private boolean SpeedToggle = true;

        DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Arm;

        CRServo FoundationClaw;
        CRServo CapDropper;

        Servo StoneClampL;
        Servo StoneClampR;

        Servo StoneLift;
        Servo StoneTurner;

        public ElapsedTime runtime = new ElapsedTime();

        public void init() {
            FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            //DriveL.setDirection(DcMotor.Direction.REVERSE);

            BackRight = hardwareMap.get(DcMotor.class, "BackRight");
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            Arm = hardwareMap.get(DcMotor.class, "Arm");

            FoundationClaw = hardwareMap.get(CRServo.class, "FoundationClaw");
            StoneClampL = hardwareMap.get(Servo.class, "StoneClampL");
            StoneClampR = hardwareMap.get(Servo.class, "StoneClampR");

            StoneLift = hardwareMap.get(Servo.class, "StoneLift");
            StoneTurner = hardwareMap.get(Servo.class, "StoneTurner");

            CapDropper = hardwareMap.get(CRServo.class, "CapDropper");
        }

        @Override
        public void loop() {
            turnspeed = gamepad1.right_stick_x * robotControlSpeed;
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
            //Turning

            if(gamepad1.x && !usedRecently){
                if(SpeedToggle){
                    SpeedToggle = false;
                }else{
                    SpeedToggle = true;
                }
                runtime.reset();
                usedRecently = true;
            }

            if(!StopperTouched) {
                if(gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1) {
                    Arm.setPower(-gamepad2.right_stick_y);
                }else{
                    Arm.setPower(0);
                }
            }else{
                Arm.setPower(0);
            }


            if(gamepad2.a){
                FoundationClaw.setPower(-1);
            }else if(gamepad2.b){
                FoundationClaw.setPower(1);
            }else{
                FoundationClaw.setPower(0.5);
            }

            if(gamepad2.x){
                StoneLift.setPosition(1);
            }else if(gamepad2.y){
                StoneLift.setPosition(0);
            }

            if(gamepad2.left_bumper){
                StoneTurner.setPosition(1);
            }else if(gamepad2.right_bumper) {
                StoneTurner.setPosition(0);
            }

            if(gamepad2.right_trigger > 0.1){

                StoneClampL.setPosition(0);
            }else if(gamepad2.left_trigger > 0.1){
                StoneClampR.setPosition(-1);
                StoneClampL.setPosition(1);
            }else{
                StoneClampR.setPosition(0.5);
            }

            if (gamepad1.b){
                CapDropper.setPower(1);
            }else{
                CapDropper.setPower(0);
            }

            //Strafing
            if (SpeedToggle) {
                FrontRight.setPower( (speed - strafespeed - turnspeed)/2 );
                BackRight.setPower( (speed + strafespeed - turnspeed)/2 );
                FrontLeft.setPower( (speed + strafespeed + turnspeed)/2 );
                BackLeft.setPower( (speed - strafespeed + turnspeed)/2 );
            }else{
                FrontRight.setPower(speed - strafespeed - turnspeed);
                BackRight.setPower(speed + strafespeed - turnspeed);
                FrontLeft.setPower(speed + strafespeed + turnspeed);
                BackLeft.setPower(speed - strafespeed + turnspeed);
            }

            if (runtime.milliseconds() > 200) {
                usedRecently = false;
            }

            telemetry.addData("Speeds: ", "%.5f, %.5f, %.5f", (speed), (strafespeed), (turnspeed));
            telemetry.update();

        }
    }



