
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Disco")
public class MicroTeleop extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    boolean activated = false;
    //SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double riseTime = 1.75;

        Pose2d poseEstimate = driveTrain.getPoseEstimate();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.b && !activated) {
                activated = true;
                reverseTimer.reset();
            } else if (gamepad1.b && activated && reverseTimer.seconds() <= riseTime) {
                double t = reverseTimer.seconds();
                driveTrain.carousel.setPower(-1 * Math.sqrt(1 - (1 / Math.pow(riseTime, 2)) * Math.pow(t - riseTime, 2)));
            } else if (gamepad1.b && activated && reverseTimer.seconds() > riseTime) {
                driveTrain.carousel.setPower(-1.0);
            }

            if (!gamepad1.x && !gamepad1.b) {
                driveTrain.carousel.setPower(0);
                activated = false;
            }

            if(gamepad2.b){
                driveTrain.sort.setPosition(0);
            }
            if(gamepad2.x){
                driveTrain.sort.setPosition(0.55);
            }
            if(gamepad2.y){
                driveTrain.sort.setPosition(1);
            }
            if (gamepad2.a) {
                driveTrain.clamp.setPosition(0);
                //externalContServoControl();
                //driveTrain.clamp.setPower(1);
            } else {
                driveTrain.clamp.setPosition(0.5);
                //driveTrain.clamp.setPower(-1);

            }

            if ((gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1)) {
                driveTrain.deposit.setPower(gamepad2.left_stick_y * 0.4);
            } else {
                driveTrain.deposit.setPower(0);
            }
            /*
            if ((gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1)) {
                driveTrain.intake.setPower(gamepad2.left_stick_y * 0.4);
            } else {
                driveTrain.intake.setPower(0);
            }*/


            driveTrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, gamepad1.right_stick_x * 0.7));
            driveTrain.update();
        }

    }
    /*
    private void externalContServoControl() {
        timer.reset();
        while(timer.milliseconds() <= 500) {
            driveTrain.clamp.setPosition(1);
        }
        while(timer.milliseconds() <= 1000){
            driveTrain.clamp.setPosition(0);
        }
        timer.reset();
    }*/
}
=======
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Disco")
public class MicroTeleop extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    boolean activated = false;
    //SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double riseTime = 1.75;

        Pose2d poseEstimate = driveTrain.getPoseEstimate();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.b && !activated) {
                activated = true;
                reverseTimer.reset();
            } else if (gamepad1.b && activated && reverseTimer.seconds() <= riseTime) {
                double t = reverseTimer.seconds();
                driveTrain.carousel.setPower(-1 * Math.sqrt(1 - (1 / Math.pow(riseTime, 2)) * Math.pow(t - riseTime, 2)));
            } else if (gamepad1.b && activated && reverseTimer.seconds() > riseTime) {
                driveTrain.carousel.setPower(-1.0);
            }

            if (!gamepad1.x && !gamepad1.b) {
                driveTrain.carousel.setPower(0);
                activated = false;
            }

            if(gamepad2.b){
                driveTrain.sort.setPosition(0);
            }
            if(gamepad2.x){
                driveTrain.sort.setPosition(0.55);
            }
            if(gamepad2.y){
                driveTrain.sort.setPosition(1);
            }
            if (gamepad2.a) {
                driveTrain.clamp.setPosition(0);
                //externalContServoControl();
                //driveTrain.clamp.setPower(1);
            } else {
                driveTrain.clamp.setPosition(0.5);
                //driveTrain.clamp.setPower(-1);

            }

            if ((gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1)) {
                driveTrain.deposit.setPower(gamepad2.left_stick_y * 0.4);
            } else {
                driveTrain.deposit.setPower(0);
            }
            /*
            if ((gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1)) {
                driveTrain.intake.setPower(gamepad2.left_stick_y * 0.4);
            } else {
                driveTrain.intake.setPower(0);
            }*/


            driveTrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, gamepad1.right_stick_x * 0.7));
            driveTrain.update();
        }

    }
    /*
    private void externalContServoControl() {
        timer.reset();
        while(timer.milliseconds() <= 500) {
            driveTrain.clamp.setPosition(1);
        }
        while(timer.milliseconds() <= 1000){
            driveTrain.clamp.setPosition(0);
        }
        timer.reset();
    }*/
}
