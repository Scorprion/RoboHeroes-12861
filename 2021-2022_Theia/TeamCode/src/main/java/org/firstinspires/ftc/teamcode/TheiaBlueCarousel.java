package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ColorScanner;
import org.firstinspires.ftc.teamcode.MarkerLocation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.TheiaDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group="Theia")
public class TheiaBlueCarousel extends LinearOpMode {
    MarkerLocation position = MarkerLocation.UNKNOWN;
    private double riseTime = 2.25;
    private double spinTime = riseTime + 0.25;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        TheiaDrive robot = new TheiaDrive(hardwareMap);
        ColorScanner pipeline = new ColorScanner(telemetry, true);
        robot.camera.setPipeline(pipeline);
        robot.setPoseEstimate(new Pose2d(-30, 64, Math.toRadians(0)));

        // Opening async. to avoid the thread from waiting for camera
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robot.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        // Start Scanning
        while(opModeIsActive() && position == MarkerLocation.UNKNOWN) {
            position = pipeline.getLocation();
        }

        robot.camera.stopStreaming();

        robot.intakearm.setPower(-0.7);
        if(position == MarkerLocation.LEFT) {
            // Level 1
            robot.sorter.setPosition(0.0);
        } else if(position == MarkerLocation.MIDDLE) {
            // Level 2
            robot.sorter.setPosition(0.5);
        } else {
            // Level 3
            robot.sorter.setPosition(1.0);
        }
        sleep(1000);

        // Move to carousel
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, 50, Math.toRadians(0)))
                .build();
        robot.followTrajectory(move);

        robot.preload.setPower(0.65);
        robot.intakearm.setPower(0.4);
        sleep(1000);
        robot.caparm.setPower(0.7);
        sleep(500);
        robot.preload.setPower(0);
        robot.caparm.setPower(0);
        robot.intakearm.setPower(0);

        // Move into carousel (to spin)
        Trajectory move2 = robot.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(-62, 58, Math.toRadians(0)))
                .build();
        robot.followTrajectory(move2);


        // Spin carousel
        timer.reset();
        double t = timer.seconds();
        while (t <= spinTime && opModeIsActive()) {
            if (t < riseTime) robot.carousel.setPower(-Math.sqrt(1 - (1.0 / (riseTime * riseTime)) * Math.pow(t - riseTime, 2)));
            else robot.carousel.setPower(-1);
            t = timer.seconds();
        }
        robot.carousel.setPower(0);

        // Move to alliance hub
        Trajectory move3 = robot.trajectoryBuilder(move2.end())
                .lineToLinearHeading(new Pose2d(-9, 42, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move3);

        // Move forward to hub
        Trajectory move4 = robot.trajectoryBuilder(move3.end())
                .lineToLinearHeading(new Pose2d(-9, 36, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move4);

        // Depositing
        robot.outtake.setPower(0.25);
        sleep(1000);
        robot.outtake.setPower(0);
        robot.release.setPower(-1);
        sleep(500);
        robot.release.setPower(0);

        // Prepare to move into depot
        Trajectory move5 = robot.trajectoryBuilder(move4.end())
                .lineToLinearHeading(new Pose2d(0, 63, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    robot.release.setPower(1);
                    robot.outtake.setPower(-0.45);
                    robot.sorter.setPosition(1.0);
                })
                .build();
        robot.followTrajectory(move5);

        robot.release.setPower(0);
        robot.outtake.setPower(0);

        // Slowly move in depot + spin intake
        Trajectory move6 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(37, 63, Math.toRadians(0)),
                        TheiaDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        TheiaDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1, () -> {
                    robot.intakearm.setPower(-0.5);
                })
                .addTemporalMarker(2, () -> {
                    robot.spintake.setPower(-1);
                    robot.intakearm.setPower(0);
                })
                .build();
        robot.followTrajectory(move6);

        // Move out of depot and move to hub
        Trajectory move7 = robot.trajectoryBuilder(move6.end())
                .lineToLinearHeading(new Pose2d(0, 63, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-9, 38, Math.toRadians(90)), Math.toRadians(-90))
                .addTemporalMarker(0.5, () -> {
                    robot.intakearm.setPower(0.5);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.spintake.setPower(0);
                    robot.intakearm.setPower(0);
                })
                .build();
        robot.followTrajectory(move7);

        // Depositing
        robot.outtake.setPower(0.25);
        sleep(1000);
        robot.outtake.setPower(0);
        robot.release.setPower(-1);
        sleep(500);
        robot.release.setPower(0);


        Trajectory move10 = robot.trajectoryBuilder(move7.end())
                .splineToSplineHeading(new Pose2d(0, 61, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(43, 63, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{
                    robot.release.setPower(1);
                    robot.outtake.setPower(-0.35);
                })
                .build();
        robot.followTrajectory(move10);
        robot.release.setPower(0);
        robot.outtake.setPower(0);
    }
}
