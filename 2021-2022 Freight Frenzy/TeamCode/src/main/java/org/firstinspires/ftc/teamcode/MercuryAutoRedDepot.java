package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class MercuryAutoRedDepot extends LinearOpMode {
    private double riseTime = 1.75;
    private double spinTime = riseTime + 0.25;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        driveTrain.setPoseEstimate(new Pose2d(12, -64, Math.toRadians(180)));

        waitForStart();

        driveTrain.wrist.setPosition(0);

        Trajectory move = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12, -42, Math.toRadians(180)))
                .build();
        driveTrain.followTrajectory(move);

        // Depositing
        driveTrain.elbow.setPower(-0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);
        driveTrain.clamp.setPosition(1);
        sleep(500);
        driveTrain.elbow.setPower(0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);

        Trajectory move2 = driveTrain.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(180)))
                .build();
        driveTrain.followTrajectory(move2);

        Trajectory move3 = driveTrain.trajectoryBuilder(move2.end())
                .lineToLinearHeading(new Pose2d(42, -50, Math.toRadians(180)))
                .build();
        driveTrain.followTrajectory(move3);

        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }
}
