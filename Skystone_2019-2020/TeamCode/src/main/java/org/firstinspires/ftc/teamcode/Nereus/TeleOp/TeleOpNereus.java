package org.firstinspires.ftc.teamcode.Nereus.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "TeleOp Nereus", group= "Nereus")
public class TeleOpNereus extends OpMode {
        //Making the slower arm and elbow toggle (driver 2)
        private ElapsedTime openClaw = new ElapsedTime();
        private ElapsedTime slowdown = new ElapsedTime();
        private boolean switchedS = false;
        private boolean usedRecently = false;
        private double controlSpeedS = 1;

        private double robotControlSpeed = 0.7;

        private double upArmSpeed = 0, downArmSpeed = 0;
        private double turnspeed = 0;
        private double speed = 0;
        private double ElbowSpeed = 0;

        private double LElbowSpeed = 0;

        DcMotor Right, Left, Arm;
        CRServo Dump;

        public void init() {
            Right = hardwareMap.get(DcMotor.class, "Right");
            Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Left = hardwareMap.get(DcMotor.class, "Left");
            Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Left.setDirection(DcMotorSimple.Direction.REVERSE);

            Arm = hardwareMap.get(DcMotor.class, "Arm");


            Dump = hardwareMap.get(CRServo.class, "Dump");
        }

        @Override
        public void loop() {
            upArmSpeed = (gamepad2.right_trigger * 0.7) * controlSpeedS;
            downArmSpeed = (gamepad2.left_trigger * 0.7) * controlSpeedS;
            if( gamepad1.left_stick_x == 0){turnspeed = gamepad1.right_stick_x * robotControlSpeed;}
            if (gamepad1.right_stick_x == 0){turnspeed = gamepad1.left_stick_x * robotControlSpeed;}
            speed = gamepad1.left_stick_y * robotControlSpeed;



        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 2         |
        |                           |
        |                           |
        -----------------------------
         */
            //The Shoulder

            //Controlling Shoulder arm speed
            if (gamepad2.a && !usedRecently) {
                if (switchedS) {
                    controlSpeedS = 1;
                    switchedS = false;
                } else if (!switchedS) {
                    controlSpeedS = 0.7;
                    switchedS = true;
                }
                slowdown.reset();
                usedRecently = true;
            }

            //The LClamp
//            if (gamepad2.right_bumper) {
//                Clamp.setPower(1);
//
//            }else if (gamepad1.left_bumper) {
//                Clamp.setPower(0);
//            }else{
//                Clamp.setPower(0.5);
//            }



        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 1         |
        |                           |
        |                           |
        -----------------------------
         */
            //Resetting Latch
            //Turning
            if ((gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) || (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1)) {
                Left.setPower(turnspeed);
                Right.setPower(-turnspeed);
            }

            //Moving
            if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
                Left.setPower(-speed);
                Right.setPower(-speed);
            }
            if (gamepad2.right_stick_y >= 0.1) {
                Arm.setPower(-1);
            }
            if (gamepad2.right_stick_y <= -0.1) {
                Arm.setPower(1);
            }
            if (gamepad2.right_stick_y >= -0.1 && gamepad1.right_stick_y <= 0.1) {
                Arm.setPower(0);
            }
            //Making the robot stop when it's set to 0
            if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
                Left.setPower(0);
                Right.setPower(0);
            }

            if (gamepad2.right_trigger > 0) {
                Dump.setPower(gamepad2.left_trigger);
            } else if (gamepad2.left_trigger > 0) {
                Dump.setPower(-gamepad2.left_trigger);
            }

            if(usedRecently && slowdown.seconds() > 1) {
                usedRecently = false;
            }

        }

    }
