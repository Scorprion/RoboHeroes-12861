package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group="Theia")
public class TheiaTeleOp extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx carousel, outtake, intakearm, caparm;
    Servo sorter;
    CRServo preload, spintake, release;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    boolean activated = false;
    private double spinTime = 2.0, riseTime = 1.75;


    @Override
    public void runOpMode() throws InterruptedException {
        double speed, turnspeed, strafespeed;

        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        carousel = hardwareMap.get(DcMotorEx.class, "Carousel");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intakearm = hardwareMap.get(DcMotorEx.class, "IntakeArm");
        caparm = hardwareMap.get(DcMotorEx.class, "CapArm");

        preload = hardwareMap.get(CRServo.class, "Preload");
        spintake = hardwareMap.get(CRServo.class, "Spintake");
        release = hardwareMap.get(CRServo.class, "Release");
        sorter = hardwareMap.get(Servo.class, "Sorter");

        DcMotorEx[] motors = { frontLeft, frontRight, backLeft, backRight };

        for(DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            speed = gamepad1.left_stick_y;
            turnspeed = gamepad1.right_stick_x;
            strafespeed = -gamepad1.left_stick_x;

            frontRight.setPower(speed + turnspeed - strafespeed);
            frontLeft.setPower( speed - turnspeed + strafespeed);
            backRight.setPower( speed + turnspeed + strafespeed);
            backLeft.setPower(  speed - turnspeed - strafespeed);

            outtake.setPower(0.35 * (gamepad2.right_stick_y));
            caparm.setPower(0.4 * -gamepad2.left_stick_y);

            if(gamepad1.b) {
                spintake.setPower(1.0);
            } else if(gamepad1.a) {
                spintake.setPower(-1.0);
            } else {
                spintake.setPower(0.0);
            }

            intakearm.setPower(0.4 * (gamepad1.right_trigger - gamepad1.left_trigger));

            if(gamepad2.a) {
                sorter.setPosition(0.0);
            } else if(gamepad2.b) {
                sorter.setPosition(0.5);
            } else {
                sorter.setPosition(1.0);
            }

            if(gamepad2.x) {
                release.setPower(1.0);
            } else if (gamepad2.y) {
                release.setPower(-1.0);
            } else {
                release.setPower(0.0);
            }

            if (gamepad1.x && !activated) {
                activated = true;
                timer.reset();
            } else if (gamepad1.x && activated && timer.seconds() <= riseTime) {
                double t = timer.seconds();
                carousel.setPower(Math.sqrt(1 - (1 / Math.pow(riseTime, 2)) * Math.pow(t - riseTime, 2)));
            } else if (gamepad1.x && activated && timer.seconds() > riseTime) {
                carousel.setPower(1.0);
            }

            if (gamepad1.b && !activated) {
                activated = true;
                reverseTimer.reset();
            } else if (gamepad1.b && activated && reverseTimer.seconds() <= riseTime) {
                double t = reverseTimer.seconds();
                carousel.setPower(-1 * Math.sqrt(1 - (1 / Math.pow(riseTime, 2)) * Math.pow(t - riseTime, 2)));
            } else if (gamepad1.b && activated && reverseTimer.seconds() > riseTime) {
                carousel.setPower(-1.0);
            }

            if (!gamepad1.x && !gamepad1.b) {
                carousel.setPower(0);
                activated = false;
            }


        }
    }
}
