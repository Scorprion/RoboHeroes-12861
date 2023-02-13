/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.ThorDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class ThorRedRightDeposit extends LinearOpMode
{
   AprilTagDetectionPipeline aprilTagDetectionPipeline;
   ThorDrive drive;

   static final double FEET_PER_METER = 3.28084;

   // Lens intrinsics
   // UNITS ARE PIXELS
   // NOTE: this calibration is for the C920 webcam at 800x448.
   // You will need to do your own calibration for other configurations!
   double fx = 578.272;
   double fy = 578.272;
   double cx = 402.145;
   double cy = 221.506;

   // UNITS ARE METERS
   double tagsize = 0.166;

   int[] ID_TAG_OF_INTEREST = {0, 1, 2} ; // Tag ID 0, 1, and 2 from the 36h11 family

   AprilTagDetection tagOfInterest = null;

   @Override
   public void runOpMode()
   {
      drive = new ThorDrive(hardwareMap);
      drive.setPoseEstimate(new Pose2d(37, -64, Math.toRadians(90)));
      
      aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

      drive.camera.setPipeline(aprilTagDetectionPipeline);
      drive.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
      {
         @Override
         public void onOpened()
         {
            drive.camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
         }

         @Override
         public void onError(int errorCode)
         {

         }
      });

      telemetry.setMsTransmissionInterval(50);

      /*
       * The INIT-loop:
       * This REPLACES waitForStart!
       */
      while (!isStarted() && !isStopRequested())
      {
         ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

         if(currentDetections.size() != 0)
         {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
               for (int j : ID_TAG_OF_INTEREST) {
                  if (tag.id == j) {
                     tagOfInterest = tag;
                     tagFound = true;
                     break;
                  }
               }
            }

            if(tagFound) {
               telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
               tagToTelemetry(tagOfInterest);
            } else {
               telemetry.addLine("Don't see tag of interest :(");

               if(tagOfInterest == null) {
                  telemetry.addLine("(The tag has never been seen)");
               } else {
                  telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                  tagToTelemetry(tagOfInterest);
               }
            }

         } else {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
               telemetry.addLine("(The tag has never been seen)");
            } else {
               telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
               tagToTelemetry(tagOfInterest);
            }
         }

         telemetry.update();
         sleep(20);
      }

      /*
       * The START command just came in: now work off the latest snapshot acquired
       * during the init loop.
       */

      /* Update the telemetry */
      if(tagOfInterest != null) {
         telemetry.addLine("Tag snapshot:\n");
         tagToTelemetry(tagOfInterest);
         telemetry.update();
      } else {
         telemetry.addLine("No april tag detected. Running to default center position.");
         telemetry.update();
      }

      drive.clampLeft.setPosition(0);
      drive.clampRight.setPosition(1);

      // Stop and reset encoders
      // Set target position (4000 counts high with starting position calibration) (0 with extended calibration, -4000 for the starting position)
      drive.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      drive.lift.setTargetPosition(1850);
      drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      drive.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      drive.pivot.setTargetPosition(133);
      drive.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      sleep(2500);

      drive.lift.setPower(0.6);

      sleep(250);

      // Deliver preload
      Trajectory startToPole = drive.trajectoryBuilder(drive.getPoseEstimate())
              .forward(38)
      //        .lineToLinearHeading(new Pose2d(37,-26, Math.toRadians(90)))
              .build();
      drive.followTrajectory(startToPole);

      // Possibly add a section to wait until its up to the correct height
      // Rotate pivot to the side, waiting 500 ms for it to do so
      drive.pivot.setPower(0.15);
      sleep(2000);
      drive.pivot.setPower(0);
      drive.lift.setPower(0);

      // Open clamp
      drive.clampLeft.setPosition(1);
      drive.clampRight.setPosition(0);

      sleep(1500);

      drive.pivot.setTargetPosition(0);
      drive.lift.setTargetPosition(0);

      // Forward 27 inches, strafe 23 inches to sides
      Trajectory center = drive.trajectoryBuilder(startToPole.end())
           .lineToLinearHeading(new Pose2d(37,-37, Math.toRadians(90)))
           .build();
      Trajectory strafeLeft = drive.trajectoryBuilder(center.end())
           .lineToLinearHeading(new Pose2d(14,-37, Math.toRadians(90)))
           .build();
      Trajectory strafeRight = drive.trajectoryBuilder(center.end())
           .lineToLinearHeading(new Pose2d(60,-37, Math.toRadians(90)))
           .build();

      // Default to the center if nothing was detected
      if(tagOfInterest == null) {
         drive.followTrajectory(center);
      } else {
         // Left position
         if(tagOfInterest.id == 0) {
            drive.followTrajectory(center);
            drive.followTrajectory(strafeLeft);
         // Right position
         } else if(tagOfInterest.id == 2) {
            drive.followTrajectory(center);
            drive.followTrajectory(strafeRight);
         // Again, default to the center in any other case
         } else {
            drive.followTrajectory(center);
         }

         /*
         // e.g.
         if(tagOfInterest.pose.x <= 20) {
            // do something
         } else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
            // do something else
         } else if(tagOfInterest.pose.x >= 50) {
            // do something else
         }*/
      }

      PoseStorage.endOfAutoPose = drive.getPoseEstimate();

      /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
      while (opModeIsActive()) {
         drive.pivot.setPower(0.15);
         drive.lift.setPower(0.4);
      }
   }

   @SuppressLint("DefaultLocale")
   void tagToTelemetry(AprilTagDetection detection)
   {
      telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
      telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
      telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
      telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
      telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
      telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
      telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
   }
}