package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class MercuryAuto extends LinearOpMode {
    double distance;
    Double minDistance = Double.POSITIVE_INFINITY;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        driveTrain.setPoseEstimate(new Pose2d(12, 72, 0));

        waitForStart();

        driveTrain.wrist.setPosition(0);

        Trajectory move = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12, 48, 0))
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
                .splineTo(new Vector2d(40, 60), Math.toRadians(90))
                .build();
        driveTrain.followTrajectory(move2);

        driveTrain.wrist.setPosition(1);
        driveTrain.elbow.setPower(-0.5);
        sleep(500);
        driveTrain.elbow.setPower(0);

        Trajectory move3 = driveTrain.trajectoryBuilder(move2.end())
                .strafeTo(new Vector2d(45, 60))
                .build();
        driveTrain.followTrajectory(move3);

        driveTrain.clamp.setPosition(0);

        sleep(500);
        driveTrain.elbow.setPower(0.5);
        sleep(500);
        driveTrain.elbow.setPower(0);

        Trajectory move4 = driveTrain.trajectoryBuilder(move3.end())
                .lineToLinearHeading(new Pose2d(-12, 48, 0))
                .build();
        driveTrain.followTrajectory(move4);

        // Depositing
        driveTrain.elbow.setPower(-0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);
        driveTrain.clamp.setPosition(1);
        sleep(500);
        driveTrain.elbow.setPower(0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);
    }

    private void goForward(SampleMecanumDrive robot, double distance) {
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .forward(distance)
                .build();
        robot.followTrajectory(move);
    }

    private void strafe(SampleMecanumDrive robot, double x, double y, double heading) {
        Trajectory strafeLeft = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
                .build();
        robot.followTrajectory(strafeLeft);
    }

    private void splineTo(SampleMecanumDrive robot, double x, double y, double heading) {
        Trajectory spline = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)), 0)
                .build();
        robot.followTrajectory(spline);
    }
}
