package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ThorDrive;

@Config
@Autonomous
public class ThorRed extends LinearOpMode {

    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        ThorDrive drive = new ThorDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        drive.followTrajectory(forward);

    }
}