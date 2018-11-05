package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "ExampleBot2019-2", group= "Pushbot")
public class New6wheelDriveTrain extends OpMode{

        public DcMotor DriveR;
        public DcMotor DriveL;
        public DcMotor ShoulderR;
        public DcMotor ShoulderL;
        double speed = 0;
        public DcMotor Elbow;
        public Servo Clamp;
        double ShoulderSpeed = 0;
        double ElbowSpeed = 0;

        //Initializing the motors for the arm




        //Set the speed of the motors when the left or right sticks are not idle

        public void init() {
            ShoulderL = hardwareMap.dcMotor.get("ShoulderL");
            Elbow = hardwareMap.dcMotor.get("Elbow");
            Clamp = hardwareMap.servo.get("Clamp");
            DriveR = hardwareMap.dcMotor.get("DriveR");
            DriveL = hardwareMap.dcMotor.get("DriveL");


            DriveL.setDirection(DcMotor.Direction.REVERSE);
            ShoulderL.setDirection(DcMotor.Direction.REVERSE);
        }

        @Override
        public void loop() {

            speed = gamepad1.left_stick_y * 1;


            if (gamepad1.right_stick_x >= 0.1) {

                DriveL.setPower(-1);
                DriveR.setPower (1);
            } else if (gamepad1.right_stick_x >= 0.9) {

                DriveL.setPower(-1);
                DriveR.setPower(1);
            }
            if (gamepad1.right_stick_x <= -0.1) {

                DriveL.setPower(1);
                DriveR.setPower(-1);
            } else if (gamepad1.right_stick_x <= -0.9) {

                DriveL.setPower(1);
                DriveR.setPower(-1);
            }
            if (gamepad1.a) {

                ShoulderL.setPower(1);
                ShoulderR.setPower(-1);
            }
            if(gamepad1.left_stick_y != 0) {
                ShoulderSpeed = gamepad1.left_stick_y * 0.75;
                ShoulderL.setPower(ShoulderSpeed);
            } else {
                ShoulderSpeed = 0;
            }

            if(gamepad1.right_stick_y != 0) {
                ElbowSpeed = gamepad1.right_stick_y * -0.5;
                Elbow.setPower(ElbowSpeed);
            } else {
                ElbowSpeed = 0;
            }

            if(gamepad1.x){
                Clamp.setPosition(1);
            }

            if(gamepad1.y) {
                Clamp.setPosition(0.5);
            }
            if(gamepad1.y) {
                Clamp.setPosition(0);
            }




            DriveL.setPower(speed);
            DriveR.setPower(speed);



        }
}
