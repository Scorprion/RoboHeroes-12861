package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class MercuryAutoBlueCarousel extends LinearOpMode {
    private double riseTime = 1.75;
    private double spinTime = riseTime + 0.25;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        driveTrain.setPoseEstimate(new Pose2d(-36, 64, 0));

        waitForStart();

        Trajectory move = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, 50, 0))
                .build();
        driveTrain.followTrajectory(move);

        Trajectory move2 = driveTrain.trajectoryBuilder(move.end())
                .lineToLinearHeading(new Pose2d(-62, 56, 0))
                .build();
        driveTrain.followTrajectory(move2);

        timer.reset();
        double t = timer.seconds();
        while (t <= spinTime && opModeIsActive()) {
            if (t < riseTime) driveTrain.carousel.setPower(Math.sqrt(1 - (1.0 / (riseTime * riseTime)) * Math.pow(t - riseTime, 2)));
            else driveTrain.carousel.setPower(1);
            t = timer.seconds();
        }
        driveTrain.carousel.setPower(0);

        Trajectory move3 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-16, 42, 0))
                .build();
        driveTrain.followTrajectory(move3);

        // Depositing
        driveTrain.elbow.setPower(-0.5);
        driveTrain.clamp.setPosition(1);
        sleep(1000);
        driveTrain.elbow.setPower(0);
        sleep(500);
        driveTrain.elbow.setPower(0.5);
        sleep(1000);
        driveTrain.elbow.setPower(0);

        Trajectory move4 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-14, 50, 0))
                .build();
        driveTrain.followTrajectory(move4);

        Trajectory move5 = driveTrain.trajectoryBuilder(move4.end())
                .lineToLinearHeading(new Pose2d(42, 48, 0))
                .build();
        driveTrain.followTrajectory(move5);

        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }
}
