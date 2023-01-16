package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Encoder;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Disabled
@TeleOp
public class ThorTeleop extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lift, pivot;
    Encoder parallelEncoder, perpendicularEncoder;
    Servo clampLeft, clampRight;


    @Override
    public void runOpMode() throws InterruptedException {
        double speed, turnspeed, strafespeed, liftspeed, pivotspeed;
        boolean setPosition = false;

        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        lift = hardwareMap.get(DcMotorEx.class, "Lift");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftDead"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "CenterDead"));

        clampLeft = hardwareMap.get(Servo.class, "ClampLeft");
        clampRight = hardwareMap.get(Servo.class, "ClampRight");

        DcMotorEx[] motors = { frontLeft, frontRight, backLeft, backRight, lift, pivot };

        for(DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            speed = -gamepad1.left_stick_y * 0.7;
            turnspeed = gamepad1.right_stick_x * 0.7;
            strafespeed = gamepad1.left_stick_x * 0.7;

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
                setPosition = true;
            } else if (gamepad2.dpad_down) {
                // Back
                pivot.setTargetPosition(300);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPosition = true;
            } else if (gamepad2.dpad_right) {
                // Side
                pivot.setTargetPosition(150);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPosition = true;
            } else if (gamepad2.dpad_left) {
                // Manual
                pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                setPosition = false;
            }


            // Pivot will run to encoder target position when instructed to, otherwise, set to the joystick power
            if (setPosition) {
                pivot.setPower(0.15);
            } else {
                pivot.setPower(pivotspeed);
            }

            if (gamepad2.y) {
                // Reset pivot encoder counts to the front
                pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.x) {
                // Reset lift counts
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            telemetry.addData("Pivot Counts", pivot.getCurrentPosition());
            telemetry.addData("Lift Counts", lift.getCurrentPosition());
            telemetry.addData("Center Counts", perpendicularEncoder.getCurrentPosition());
            telemetry.addData("Left Counts", parallelEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
