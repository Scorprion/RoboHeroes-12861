package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class AutoA extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        goForward(driveTrain, -37);
        // Scan here



    }

    private void goForward(SampleMecanumDrive robot, double distance) {
        Trajectory move = robot.trajectoryBuilder(new Pose2d(Math.sqrt(10368) - 3.4, -Math.sqrt(10368) + 12.5, Math.toRadians(180)))
                .forward(distance)
                .build();
        robot.followTrajectory(move);
    }
}
