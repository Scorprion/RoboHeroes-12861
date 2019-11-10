package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "TeleOp Hermes", group= "Pushbot")
public class TeleOpHermes extends OpMode {
    //Making the slower arm and elbow toggle (driver 2)
    private ElapsedTime openClaw = new ElapsedTime();
    private boolean switchedS = false;
    private boolean usedRecently = false;
    private double controlSpeedE = 1, controlSpeedS = 1;

    //Making the slower robot toggle (driver 1)
    private ElapsedTime rmove = new ElapsedTime();

    private boolean robotCSpeed = false; // the boolean for the robot's speed to be able to slow it down
    private boolean robotUsedRecent = false;
    private double robotControlSpeed = 0.7;

    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;

    DcMotor BackLeft, BackRight, FrontLeft, FrontRight;

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
    }

    @Override
    public void loop() {
        turnspeed = gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
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
        //Controlling Shoulder arm speed
        if (gamepad2.a && !usedRecently) {
            if (switchedS) {
                controlSpeedS = 1;
                controlSpeedE = 1;
                switchedS = false;
            } else if (!switchedS) {
                controlSpeedS = 0.7;
                controlSpeedE = 0.7;
                switchedS = true;
            }
            usedRecently = true;
        }
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
                FrontLeft.setPower(turnspeed);
                BackLeft.setPower(turnspeed);
                FrontRight.setPower(-turnspeed);
                BackRight.setPower(-turnspeed);
        }

        //Strafing
        if (gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) {
            FrontRight.setPower(-strafespeed);
            BackRight.setPower(strafespeed);
            FrontLeft.setPower(strafespeed);
            BackLeft.setPower(-strafespeed);
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            FrontLeft.setPower(-speed);
            BackLeft.setPower(-speed);
            FrontRight.setPower(-speed);
            BackRight.setPower(-speed);
        }

        //Making the robot stop when it's set to 0
        if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }

        //Tilting and resetting back our marker servo platform
        //The winch

        if (gamepad1.x && !robotUsedRecent) {
            if (robotCSpeed) {
                robotControlSpeed = 0.7;
                robotCSpeed = false;
            } else if (!robotCSpeed) {
                robotControlSpeed = 0.4;
                robotCSpeed = true;
            }
            robotUsedRecent = true;
            rmove.reset();
        }



        // Setting the used recently boolean to true after 200
        // milliseconds after the a button was pressed

        // Setting the LClamp power to 0.5 after the open claw is greater than 250 milliseconds

    }

}
