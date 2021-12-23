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

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double riseTime = 1.75;

        Pose2d poseEstimate = driveTrain.getPoseEstimate();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.8, gamepad1.left_stick_x * 0.8, gamepad1.right_stick_x * 0.8));
            driveTrain.update();
        }
    }
}
