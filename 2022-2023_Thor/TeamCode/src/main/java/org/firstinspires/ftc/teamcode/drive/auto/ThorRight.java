package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ThorDrive;

@Config
@Autonomous
public class ThorRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ThorDrive drive = new ThorDrive(hardwareMap);
        // drive.setPoseEstimate(new Pose2d(37, -64, Math.toRadians(90)));

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(32)
        //         .splineToConstantHeading(new Vector2d(37,-32), Math.toRadians(90))
                .build();

        waitForStart();

        drive.clampLeft.setPosition(0);
        drive.clampRight.setPosition(1);

        drive.followTrajectory(forward);

    }
}