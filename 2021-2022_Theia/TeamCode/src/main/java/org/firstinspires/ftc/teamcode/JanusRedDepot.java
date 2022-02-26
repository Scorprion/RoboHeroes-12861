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
public class JanusRedDepot extends LinearOpMode {
   MarkerLocation position = MarkerLocation.UNKNOWN;
   public double hubXDistance = -12;
   public double depotDistance = 50;

   ElapsedTime timer = new ElapsedTime();

   @Override
   public void runOpMode() throws InterruptedException {
      JanusDrive robot = new JanusDrive(hardwareMap);
      ColorScanner pipeline = new ColorScanner(telemetry, false);
      robot.camera.setPipeline(pipeline);
      robot.setPoseEstimate(new Pose2d(15, -65, Math.toRadians(180)));

      // Opening async. to avoid the thread from waiting for camera
      robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
            robot.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
         }

         @Override
         public void onError(int errorCode) {
            /*
             * This will be called if the camera could not be opened
             */
         }
      });

      waitForStart();
      timer.reset();

      // Start Scanning
      while (opModeIsActive() && position == MarkerLocation.UNKNOWN) {
         position = pipeline.getLocation();
      }

      robot.camera.stopStreaming();

      if (position == MarkerLocation.LEFT) {
         // Level 1
         robot.sorter.setPosition(0.0);
      } else if (position == MarkerLocation.MIDDLE) {
         // Level 2
         robot.sorter.setPosition(0.4);
      } else {
         // Level 3
         robot.sorter.setPosition(1.0);
      }
      sleep(500);

      Trajectory move1 = robot.trajectoryBuilder(robot.getPoseEstimate())
              .lineToLinearHeading(new Pose2d(hubXDistance, -41, Math.toRadians(270)))
              .addDisplacementMarker(pathLength -> pathLength * 0.1, () -> {
                 robot.intakearm.setPower(-0.5);
              })
              .addDisplacementMarker(p -> p * 0.5, () -> {
                  robot.preload.setPower(0.8);
               })
              .build();
      robot.followTrajectory(move1);
      // Depositing the cube
      robot.preload.setPower(0);
      robot.intakearm.setPower(0);
      robot.caparm.setPower(0.5);
      robot.outtake.setPower(0.25);
      sleep(500);
      robot.caparm.setPower(0);
      sleep(500);
      robot.outtake.setPower(0);
      robot.release.setPower(-1);
      sleep(500);
      robot.release.setPower(0);

      while (opModeIsActive() && timer.seconds() <= 20) {
         robot.sorter.setPosition(0.9);
         robot.intakearm.setPower(0.55);
         // Drive into the depot and spin the intake
         Trajectory move3 = robot.trajectoryBuilder(move1.end())
                 .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(-30))
                 .splineToLinearHeading(new Pose2d(20, -65, Math.toRadians(0)), Math.toRadians(0),
                         JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                         JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .addDisplacementMarker(pathLength -> pathLength * 0.05,  () -> {
                    robot.release.setPower(1);
                    robot.outtake.setPower(-0.4);
                 })
                 .splineToConstantHeading(new Vector2d(depotDistance, -65), Math.toRadians(0),
                         JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                         JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .addDisplacementMarker(pathLength -> pathLength * 0.4, () -> {
                    robot.release.setPower(0);
                    robot.spintake.setPower(-1);
                    robot.intakearm.setPower(-0.3);
                 })
                 .build();
         robot.followTrajectory(move3);

         // Slowly move out of the the depot with cube
         Trajectory move4 = robot.trajectoryBuilder(move3.end())
                 .lineToLinearHeading(new Pose2d(24, -65, Math.toRadians(0)),
                         JanusDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .splineTo(new Vector2d(hubXDistance, -41), Math.toRadians(90),
                         JanusDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .addDisplacementMarker(pathLength -> pathLength * 0.3, () -> {
                    robot.intakearm.setPower(0.45);
                 })
                 .addDisplacementMarker(pathLength -> pathLength * 0.99, () -> {
                    robot.outtake.setPower(0.25);
                 })
                 .build();
         robot.followTrajectory(move4);
         sleep(250);

         robot.release.setPower(-1);
         sleep(1000);
         robot.spintake.setPower(0);
         depotDistance += 3;
      }

      robot.intakearm.setPower(0.3);
      Trajectory move8 = robot.trajectoryBuilder(robot.getPoseEstimate())
              .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(-30))
              .splineToLinearHeading(new Pose2d(16, -65, Math.toRadians(0)), Math.toRadians(0),
                      JanusDrive.getVelocityConstraint(30, 25, DriveConstants.TRACK_WIDTH),
                      JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .splineToConstantHeading(new Vector2d(45, -65), Math.toRadians(0))
              .addDisplacementMarker(pathLength -> pathLength * 0.01, () -> {
                 robot.outtake.setPower(-0.4);
              })
              .build();
      robot.followTrajectory(move8);
      robot.outtake.setPower(0);
   }
}
