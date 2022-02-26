package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.JanusDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group="Janus")
public class JanusBlueCarouselRemote extends LinearOpMode {
    MarkerLocation position = MarkerLocation.UNKNOWN;
    private double riseTime = 2.75;
    private double spinTime = riseTime + 0.25;
    public double hubdistance = 40.0;
    public double hubXDistance = -7;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        JanusDrive robot = new JanusDrive(hardwareMap);
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
            hubdistance = 39.0;
            hubXDistance = -6.0;
        } else if(position == MarkerLocation.MIDDLE) {
            // Level 2
            robot.sorter.setPosition(0.4);
            hubdistance = 39;
            hubXDistance = -7;
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
        sleep(1500);
        robot.preload.setPower(0);
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
                .lineToLinearHeading(new Pose2d(-7, 48, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move3);

        // Move forward to hub
        Trajectory move4 = robot.trajectoryBuilder(move3.end())
                .lineToLinearHeading(new Pose2d(hubXDistance, hubdistance, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move4);

        // Depositing
        robot.caparm.setPower(0.6);
        robot.outtake.setPower(0.25);
        sleep(500);
        robot.caparm.setPower(0);
        sleep(500);
        robot.outtake.setPower(0);
        robot.release.setPower(-1);
        sleep(500);
        robot.release.setPower(0);

        // Prepare to move into depot
        Trajectory move5 = robot.trajectoryBuilder(move4.end())
                .lineToLinearHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    robot.release.setPower(1);
                    robot.outtake.setPower(-0.45);
                    robot.sorter.setPosition(1.0);
                })
                .build();
        robot.followTrajectory(move5);

        robot.outtake.setPower(0);
        sleep(300);
        robot.release.setPower(0);


        robot.spintake.setPower(-1);
        // Slowly move in depot + spin intake
        Trajectory move6 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(41, 64, Math.toRadians(0)),
                        JanusDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1, () -> {
                    robot.intakearm.setPower(-0.5);
                })
                .addTemporalMarker(2, () -> {
                    robot.intakearm.setPower(0);
                })
                .build();
        robot.followTrajectory(move6);

        // Slowly move out of depot and move to hub
        Trajectory move7 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, 64, Math.toRadians(0)),
                        JanusDrive.getVelocityConstraint(19, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(move7);

        Trajectory move8 = robot.trajectoryBuilder(move7.end())
                .lineToLinearHeading(new Pose2d(-11, 40, Math.toRadians(90)))
                .addTemporalMarker(0.5, () -> {
                    robot.intakearm.setPower(0.4);
                })
                .addTemporalMarker(2.5, () -> {
                    robot.spintake.setPower(0);
                    robot.intakearm.setPower(0);
                })
                .build();

        robot.followTrajectory(move8);
        // Depositing
        robot.outtake.setPower(0.35);
        sleep(1000);
        robot.outtake.setPower(0);
        robot.release.setPower(-1);
        sleep(500);
        robot.release.setPower(0);


        Trajectory move9 = robot.trajectoryBuilder(move8.end())
                .splineToSplineHeading(new Pose2d(0, 61, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0.5, () ->{
                    robot.release.setPower(1);
                    robot.outtake.setPower(-0.35);
                })
                .build();
        robot.followTrajectory(move9);

        robot.spintake.setPower(-1);
        Trajectory move10 = robot.trajectoryBuilder(move9.end())
                .lineToLinearHeading(new Pose2d(43, 63, Math.toRadians(0)))
                .addTemporalMarker(1, () -> {
                    robot.intakearm.setPower(-0.5);
                })
                .addTemporalMarker(2, () -> {
                    robot.intakearm.setPower(0);
                })
                .build();

        robot.followTrajectory(move10);
        robot.spintake.setPower(0);
        robot.release.setPower(0);
        robot.outtake.setPower(0);
    }
}
