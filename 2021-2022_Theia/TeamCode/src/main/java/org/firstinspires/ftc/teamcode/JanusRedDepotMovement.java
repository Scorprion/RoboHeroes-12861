package org.firstinspires.ftc.teamcode;

import android.net.wifi.p2p.WifiP2pManager;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.JanusDrive;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Disabled
@Autonomous(group="Janus")
public class JanusRedDepotMovement extends LinearOpMode {
    MarkerLocation position = MarkerLocation.UNKNOWN;
    public double hubXDistance = -12;
    public double depotDistance = 50;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        JanusDrive robot = new JanusDrive(hardwareMap);
        robot.setPoseEstimate(new Pose2d(15, -65, Math.toRadians(180)));

        waitForStart();
        timer.reset();

        Trajectory move1 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(hubXDistance, -41, Math.toRadians(270)))
                .build();
        robot.followTrajectory(move1);

        while (opModeIsActive() && timer.seconds() <= 20) {
            // Drive into the depot and spin the intake
            Trajectory move3 = robot.trajectoryBuilder(move1.end())
                    .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(-30))
                    .splineToLinearHeading(new Pose2d(20, -66, Math.toRadians(0)), Math.toRadians(0),
                            JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                            JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(new Vector2d(depotDistance, -66), Math.toRadians(0),
                            JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                            JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectory(move3);

            // Slowly move out of the the depot with cube
            Trajectory move4 = robot.trajectoryBuilder(move3.end())
                    .lineToLinearHeading(new Pose2d(hubXDistance, -66, Math.toRadians(0)),
                            JanusDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectory(move4);

            Trajectory move5 = robot.trajectoryBuilder(move4.end())
                    .strafeTo(new Vector2d(hubXDistance, -70),
                            JanusDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectory(move5);
            robot.setPoseEstimate(new Pose2d(hubXDistance, -66, Math.toRadians(0)));

            Trajectory move6 = robot.trajectoryBuilder(move4.end())
                    .lineToLinearHeading(new Pose2d(hubXDistance, -41, Math.toRadians(270)),
                            JanusDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectory(move6);
            depotDistance += 3;
            sleep(500);
        }

        Trajectory move8 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(16, -66, Math.toRadians(0)), Math.toRadians(0),
                        JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                        JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(45, -66   ), Math.toRadians(0))
                .build();
        robot.followTrajectory(move8);
    }
}
