package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "New6wheelDriveTrain", group= "Pushbot")
@Disabled
public class New6wheelDriveTrain extends OpMode {

    public DcMotor DriveR;
    public DcMotor DriveL;
    public DcMotor ShoulderR;
    public DcMotor ShoulderL;
    public DcMotor ElbowL;
    public DcMotor Winch1;
    public DcMotor Winch2;
    public Servo Clamp;
    double ShoulderSpeed = 0;
    double ElbowSpeed = 0;
    double turnspeed = 0;
    double speed = 0;

    //Initializing the motors for the arm


    //Set the speed of the motors when the left or right sticks are not idle

    public void init() {
        ShoulderL = hardwareMap.dcMotor.get("ShoulderL");
        ElbowL = hardwareMap.dcMotor.get("ElbowL");
        Clamp = hardwareMap.servo.get("Clamp");
        DriveR = hardwareMap.dcMotor.get("DriveR");
        DriveL = hardwareMap.dcMotor.get("DriveL");
        /*Winch1 = hardwareMap.dcMotor.get("Winch1");
        Winch2 = hardwareMap.dcMotor.get("Winch2");*/


        //DriveL.setDirection(DcMotor.Direction.REVERSE);
        ShoulderL.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        turnspeed = gamepad1.left_stick_x * 1;
        speed = gamepad1.left_stick_y * 1;
        telemetry.addData("The speed for both motors", speed);
        telemetry.addData("The speed for both motors in turning", turnspeed);
        //Turning
        if (gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) {
            DriveL.setPower(turnspeed);
            DriveR.setPower(turnspeed * 0.99999);
        } else {
            DriveL.setPower(0);
            DriveR.setPower(0);
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            DriveL.setPower(-speed);
            DriveR.setPower(speed * 0.99999);
        } else {
            DriveL.setPower(0);
            DriveR.setPower(0);
        }

        //The shoulder
        if (gamepad2.left_stick_y != 0) {
            ShoulderSpeed = gamepad2.left_stick_y * 0.75;
            ShoulderL.setPower(ShoulderSpeed);
        } else {
            ShoulderSpeed = 0;
        }

        //The elbow
        if (gamepad2.right_stick_y != 0) {
            ElbowSpeed = gamepad2.right_stick_y * -0.5;
            ElbowL.setPower(ElbowSpeed);
        } else {
            ElbowSpeed = 0;
        }

        if (gamepad1.a) {
            DriveR.setPower(1);
        } else {
            DriveR.setPower(0);
        }

        //The clamp
        if (gamepad2.x) {
            Clamp.setPosition(1);
        }
        if (gamepad2.y) {
            Clamp.setPosition(0.5);
        }
        if (gamepad2.a) {
            Clamp.setPosition(0);
        }

        //The winch
        /*if (gamepad2.left_bumper) {
            Winch1.setPower(1);
            Winch2.setPower(1);
        } else {
            Winch1.setPower(0);
            Winch2.setPower(0);


        }*/

    }
}
