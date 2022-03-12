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
import org.firstinspires.ftc.teamcode.drive.JanusDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group="Janus")
public class JanusRedCarousel extends LinearOpMode {
    MarkerLocation position = MarkerLocation.UNKNOWN;
    private double riseTime = 2.75;
    private double spinTime = riseTime + 0.25;
    public double hubdistance = 44.0;
    public double hubXDistance = -16;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        JanusDrive robot = new JanusDrive(hardwareMap);
        ColorScanner pipeline = new ColorScanner(telemetry, false);
        robot.camera.setPipeline(pipeline);
        robot.setPoseEstimate(new Pose2d(-30, -64, Math.toRadians(180)));

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
        while (opModeIsActive() && position == MarkerLocation.UNKNOWN && timer.seconds() < 1) {
            position = pipeline.getLocation();
        }

        robot.camera.stopStreaming();

        robot.intakearm.setPower(-0.7);
        if (position == MarkerLocation.LEFT) {
            // Level 1
            robot.sorter.setPosition(0.0);
            hubdistance = 44.0;
            hubXDistance = -17;
        } else if (position == MarkerLocation.MIDDLE) {
            // Level 2
            robot.sorter.setPosition(0.4);
            hubdistance = 44.0;
            hubXDistance = -16.5;
        } else {
            // Level 3
            robot.sorter.setPosition(1.0);
        }
        sleep(1000);


        // Move to carousel
        robot.intakearm.setPower(0.37);
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, -50, Math.toRadians(90)),
                        JanusDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        JanusDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(pathLength -> pathLength * 0.3, () -> {
                    robot.preload.setPower(0.6);
                })
                .addDisplacementMarker(p -> p * 0.9, () -> {
                    robot.preload.setPower(0);
                })
                .build();
        robot.followTrajectory(move);

        // Move into carousel (to spin)
        Trajectory move2 = robot.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(-62, -60, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move2);

        robot.setPoseEstimate(new Pose2d(-62, -56, Math.toRadians(90)));

        // Spin carousel
        timer.reset();
        double t = timer.seconds();
        while (t <= spinTime && opModeIsActive()) {
            if (t < riseTime)
                robot.carousel.setPower(Math.sqrt(1 - (1.0 / (riseTime * riseTime)) * Math.pow(t - riseTime, 2)));
            else robot.carousel.setPower(1);
            t = timer.seconds();
        }
        robot.carousel.setPower(0);

        robot.release.setPower(1);
        sleep(500);
        robot.release.setPower(0);

        Trajectory move20 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-55, -20, Math.toRadians(90)))
                .build();
        robot.followTrajectory(move20);

        // Move to alliance hub
        Trajectory move3 = robot.trajectoryBuilder(move20.end())
                .lineToLinearHeading(new Pose2d(-27, -22    , Math.toRadians(180)))
                .addDisplacementMarker(p -> p * 0.7, () -> {
                    robot.outtake.setPower(0.2);
                })
                .build();
        robot.followTrajectory(move3);

        // Depositing
        robot.caparm.setPower(0.6);
        sleep(500);
        robot.caparm.setPower(0);
        sleep(500);
        robot.outtake.setPower(0);
        sleep(1000);
        robot.release.setPower(-1);
        sleep(500);
        robot.release.setPower(0);
        robot.intakearm.setPower(0.4);
        robot.outtake.setPower(-0.4);
        sleep(500);

        // Park in the freightDepot
        robot.intakearm.setPower(0.37);
        Trajectory move10 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-58, -22, Math.toRadians(0)))
                .addDisplacementMarker(p -> p * 0.05, () -> {
                    robot.outtake.setPower(-0.4);
        })
                .build();
        robot.followTrajectory(move10);

        Trajectory move11 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(0)))
                .addDisplacementMarker(p -> p * 0.5, () -> {
                    robot.release.setPower(1);
                })
                .build();
        robot.followTrajectory(move11);
        robot.release.setPower(0);
        robot.sorter.setPosition(0.9);
        robot.preload.setPower(-0.4);
        sleep(250);
        robot.preload.setPower(0);
        sleep(100000);  // to keep the arm up
    }
}