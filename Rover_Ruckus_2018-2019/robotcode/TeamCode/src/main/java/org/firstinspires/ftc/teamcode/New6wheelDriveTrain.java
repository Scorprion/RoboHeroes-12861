package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "New6wheelDriveTrain", group= "Pushbot")
public class New6wheelDriveTrain extends OpMode{

        public DcMotor DriveR;
        public DcMotor DriveL;
        public DcMotor ShoulderR;
        public DcMotor ShoulderL;
        public DcMotor ElbowL;
        public Servo Clamp;
        double ShoulderSpeed = 0;
        double ElbowSpeed = 0;
        double speed = 0;

        //Initializing the motors for the arm




        //Set the speed of the motors when the left or right sticks are not idle

        public void init() {
            ShoulderL = hardwareMap.dcMotor.get("ShoulderL");
            ElbowL = hardwareMap.dcMotor.get("ElbowL");
            Clamp = hardwareMap.servo.get("Clamp");
            DriveR = hardwareMap.dcMotor.get("DriveR");
            DriveL = hardwareMap.dcMotor.get("DriveL");


            DriveL.setDirection(DcMotor.Direction.REVERSE);
            ShoulderL.setDirection(DcMotor.Direction.REVERSE);
        }

        @Override
        public void loop() {

            if (gamepad1.left_stick_y != 0) {
                speed = gamepad1.left_stick_y * 1;
                DriveL.setPower(-speed);
                DriveR.setPower (speed);
            }else if (gamepad1.left_stick_x >= -0.1 || gamepad1.left_stick_x <= -0.9) {
                speed = gamepad1.left_stick_y * 1;
                DriveL.setPower(speed);
                DriveR.setPower (-speed);
            } else {
                DriveL.setPower(0);
                DriveR.setPower(0);
                speed = 0;
            }

            if(gamepad2.left_stick_y != 0) {
                ShoulderSpeed = gamepad2.left_stick_y * 0.75;
                ShoulderL.setPower(ShoulderSpeed);
            } else {
                ShoulderSpeed = 0;
            }

            if(gamepad1.a) {
                DriveR.setPower(1);
            } else {
                DriveR.setPower(0);
            }

            if(gamepad2.right_stick_y != 0) {
                ElbowSpeed = gamepad2.right_stick_y * -0.5;
                ElbowL.setPower(ElbowSpeed);
            } else {
                ElbowSpeed = 0;
            }

            if(gamepad2.x){
                Clamp.setPosition(1);
            }

            if(gamepad2.y) {
                Clamp.setPosition(0.5);
            }
            if(gamepad2.a) {
                Clamp.setPosition(0);
            }
            if(gamepad1.dpad_down) {
                ElbowSpeed *= 0.5;
            }


            if(gamepad1.left_stick_y !=0 ) {
                DriveL.setPower(speed);
                DriveR.setPower(speed);
            } else{
                speed = 0;
                DriveL.setPower(0);
                DriveR.setPower(0);
            }

        }
}
