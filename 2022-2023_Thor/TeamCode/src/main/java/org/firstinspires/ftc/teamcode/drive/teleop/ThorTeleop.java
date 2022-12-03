package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ThorTeleop extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lift, pivot;
    Servo clampLeft, clampRight;

    @Override
    public void runOpMode() throws InterruptedException {
        double speed, turnspeed, strafespeed, liftspeed, pivotspeed;

        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        lift = hardwareMap.get(DcMotorEx.class, "Lift");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");

        clampLeft = hardwareMap.get(Servo.class, "ClampLeft");
        clampRight = hardwareMap.get(Servo.class, "ClampRight");

        DcMotorEx[] motors = { frontLeft, frontRight, backLeft, backRight, lift, pivot };

        for(DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            speed = -gamepad1.left_stick_y;
            turnspeed = gamepad1.right_stick_x;
            strafespeed = gamepad1.left_stick_x;

            liftspeed = -gamepad2.left_stick_y;
            pivotspeed = gamepad2.right_stick_x * 0.2;

            frontRight.setPower(speed - turnspeed - strafespeed);
            frontLeft.setPower(speed + turnspeed + strafespeed);
            backRight.setPower(speed - turnspeed + strafespeed);
            backLeft.setPower(speed + turnspeed - strafespeed);

            lift.setPower(liftspeed);


            if (gamepad2.b) {
                // Open
                clampLeft.setPosition(0.6);
                clampRight.setPosition(0.3);
            } else if (gamepad2.a) {
                // Clamp
                clampLeft.setPosition(0);
                clampRight.setPosition(1);
            }

            if (gamepad2.dpad_up) {
                // Front
                pivot.setTargetPosition(0);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(0.2);
            } else if (gamepad2.dpad_down) {
                // Back
                pivot.setTargetPosition(300);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(0.2);
            } else if (gamepad2.dpad_right) {
                // Side
                pivot.setTargetPosition(150);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(0.2);
            } else {
                // Manual
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivot.setPower(pivotspeed);
            }


            telemetry.addData("Counts", pivot.getCurrentPosition());
            telemetry.update();
        }
    }
}
