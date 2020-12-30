package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="Disco")
public class AutoA extends LinearOpMode {
    double distance;
    Double minDistance = Double.POSITIVE_INFINITY;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        // Move towards rings
        driveTrain.ringArm.setPower(-1.0);
        strafe(driveTrain, -33);

        // TODO Run continuously while moving?
        timer.reset();
        while(timer.seconds() < 1) {
            distance = driveTrain.ringSensor.getDistance(DistanceUnit.INCH);
            if(distance < minDistance) {
                minDistance = distance;
            }
        }

        telemetry.addData("Current min", minDistance);
        telemetry.update();

        driveTrain.ringArm.setPower(1.0);
        sleep(1250);
        driveTrain.ringArm.setPower(0);


        if(minDistance >= 5.5) {  // None
            telemetry.addLine("NONE");
        } else if (minDistance >= 3) {  // SINGLE
            telemetry.addLine("SINGLE");
        } else {  // FOUR
            telemetry.addLine("FOUR");
        }
        telemetry.update();

        // 41 and go 90
        /*
        Trajectory centering = driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, -41, Math.toRadians(90)))
                .build();
        driveTrain.followTrajectory(centering);
        */
        strafe(driveTrain, -41);
        driveTrain.turn(90);

        sleep(3000);


        // Based on rings, go to a certain position


    }

    private void goForward(SampleMecanumDrive robot, double distance) {
        Trajectory move = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(distance)
                .build();
        robot.followTrajectory(move);
    }

    private void strafe(SampleMecanumDrive robot, double distance) {
        Trajectory strafeLeft = robot.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(0, distance))
                .build();
        robot.followTrajectory(strafeLeft);
    }
}
