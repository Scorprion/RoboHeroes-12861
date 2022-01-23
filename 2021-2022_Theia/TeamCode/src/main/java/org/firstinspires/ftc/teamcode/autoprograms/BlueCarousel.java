package org.firstinspires.ftc.teamcode.autoprograms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ColorScanner;
import org.firstinspires.ftc.teamcode.ColorScanner.MarkerLocation;
import org.firstinspires.ftc.teamcode.drive.TheiaDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group="Theia")
public class BlueCarousel extends LinearOpMode {
    MarkerLocation position = MarkerLocation.UNKNOWN;
    private double riseTime = 1.75;
    private double spinTime = riseTime + 0.25;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        TheiaDrive robot = new TheiaDrive(hardwareMap);
        ColorScanner pipeline = new ColorScanner(telemetry, true);
        robot.camera.setPipeline(pipeline);
        robot.setPoseEstimate(new Pose2d(-36, 64, 0));

        // Opening async. to avoid the thread from waiting for camera
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robot.camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
        while(opModeIsActive() && position == MarkerLocation.UNKNOWN) {
            position = pipeline.getLocation();
        }

        // Move to carousel
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, 50, 0))
                .build();
        robot.followTrajectory(move);

        Trajectory move2 = robot.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(-62, 56, 0))
                .build();
        robot.followTrajectory(move2);


        // Spin carousel
        timer.reset();
        double t = timer.seconds();
        while (t <= spinTime && opModeIsActive()) {
            if (t < riseTime) robot.carousel.setPower(Math.sqrt(1 - (1.0 / (riseTime * riseTime)) * Math.pow(t - riseTime, 2)));
            else robot.carousel.setPower(1);
            t = timer.seconds();
        }
        robot.carousel.setPower(0);

        // Move to alliance hub
        Trajectory move3 = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-16, 42, 0))
                .build();
        robot.followTrajectory(move3);


    }
}
