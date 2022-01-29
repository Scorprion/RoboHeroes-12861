package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class MercuryAutoBlueDepot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        driveTrain.setPoseEstimate(new Pose2d(12, 64, 0));

        waitForStart();

        Trajectory move = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12, 42, 0))
                .build();
        driveTrain.followTrajectory(move);

        // Depositing
        driveTrain.elbow.setPower(-0.5);
        driveTrain.clamp.setPosition(1);
        sleep(1000);
        driveTrain.elbow.setPower(0);
        sleep(500);
        driveTrain.elbow.setPower(0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);

        Trajectory move2 = driveTrain.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(46, 48, 0))
                .build();
        driveTrain.followTrajectory(move2);

        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }
}
