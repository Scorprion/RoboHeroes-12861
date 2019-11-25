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

        DcMotor BackLeft, BackRight, FrontLeft, FrontRight, Arm;

        RevTouchSensor Stopper;

        CRServo FoundationClaw;
        CRServo StoneClamp;

        Servo StoneLift;
        Servo StoneTurner;

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

            Stopper = hardwareMap.get(RevTouchSensor.class, "ArmStopper");

            FoundationClaw = hardwareMap.get(CRServo.class, "FoundationClaw");
            StoneClamp = hardwareMap.get(CRServo.class, "StoneClamp");

            StoneLift = hardwareMap.get(Servo.class, "StoneLift");
            StoneTurner = hardwareMap.get(Servo.class, "StoneTurner");
        }

        @Override
        public void loop() {
            turnspeed = gamepad1.right_stick_x * robotControlSpeed;
            strafespeed = gamepad1.left_stick_x * robotControlSpeed;
            speed = gamepad1.left_stick_y * -robotControlSpeed;
            StopperTouched = Stopper.isPressed();
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
            if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
                FrontRight.setPower(-turnspeed);
                BackRight.setPower(-turnspeed);
                FrontLeft.setPower(turnspeed);
                BackLeft.setPower(turnspeed);
            }

            if(!StopperTouched) {
                if(gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= -0.1) {
                    Arm.setPower(gamepad1.right_stick_y);
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
                StoneClamp.setPower(1);
            }else if(gamepad2.left_trigger > 0.1){
                StoneClamp.setPower(-1);
            }else{
                StoneClamp.setPower(0.5);
            }

            //Strafing
            FrontRight.setPower(speed -strafespeed);
            BackRight.setPower(speed + strafespeed);
            FrontLeft.setPower(speed + strafespeed);
            BackLeft.setPower(speed -strafespeed);

            telemetry.addData("Speeds: ", "%.5f, %.5f, %.5f", (speed), (strafespeed), (turnspeed));
            telemetry.update();

        }
    }



