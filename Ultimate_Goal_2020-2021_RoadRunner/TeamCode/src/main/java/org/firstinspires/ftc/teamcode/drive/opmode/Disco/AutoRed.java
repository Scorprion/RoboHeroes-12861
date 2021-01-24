package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.MatrixUtils;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DiscoDrive;

@Autonomous(group="Disco")
public class AutoRed extends LinearOpMode {
    double distance;
    long shootduration = 0;

    Double minDistance = Double.POSITIVE_INFINITY;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DiscoDrive driveTrain = new DiscoDrive(hardwareMap);

        waitForStart();

        // Lowers the distance sensor
        driveTrain.ringArm.setPower(-1.0);
        // Move towards rings
        strafe(driveTrain, 0, -33, 0);

        // TODO Run continuously while moving?
        // Using distance sensor to scan for the ring counts
        timer.reset();
        while (timer.seconds() < 1) {
            distance = driveTrain.ringSensor.getDistance(DistanceUnit.INCH);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        telemetry.addData("Current min", minDistance);
        telemetry.update();


        // Scans the rings for the wobble goal placement
        if (minDistance >= 4.6) {  // None
            telemetry.addLine("NONE");
            A(driveTrain);
        } else if (minDistance >= 3) {  // SINGLE
            telemetry.addLine("SINGLE");
            B(driveTrain);
        } else {  // FOUR
            telemetry.addLine("FOUR");
            C(driveTrain);
        }
        telemetry.update();
    }

    // Method that drops the wobble goal in zone A and shoots the rings
    private void A(DiscoDrive robot){
        // Driving to Zone A
        robot.ringArm.setPower(1.0);
        robot.turn(Math.toRadians(90));
        strafe(robot, -30, -75, 90);
        robot.wobbleSet.setPower(1);
        sleep(200);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        robot.wobbleGrab.setPosition(1);
        sleep(500);
        robot.wobbleGrab.setPosition(0);
        sleep(500);

        // Reposition to the shooting position
        strafe(robot, -30, -60, 90);
        strafe(robot, -15, -60, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,-0.3,500);
        ringout(robot, 0.52);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0);

        strafe(robot, -40, -20, 270);
        strafe(robot,-40, -5, 270);
        sleep(500);
        robot.wobbleGrab.setPosition(1);
        sleep(500);
        robot.wobbleSet.setPower(-0.7);
        sleep(1000);
        robot.wobbleSet.setPower(0);

        strafe(robot, -25, -20, 270);
        strafe(robot, -25, -75, 90);
        robot.wobbleSet.setPower(1);
        sleep(500);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        robot.wobbleGrab.setPosition(0);
        sleep(500);
        strafe(robot, 0, -75, 0);


    }

    private void B(DiscoDrive robot) {
        // Driving to Zone A
        robot.ringArm.setPower(1.0);
        robot.turn(Math.toRadians(90));
        strafe(robot, 0, -95, 90);
        robot.wobbleSet.setPower(1);
        sleep(200);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        robot.wobbleGrab.setPosition(1);
        sleep(500);
        robot.wobbleGrab.setPosition(0);
        sleep(500);

        // Reposition to the shooting position
        strafe(robot, -15, -60, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,-0.3,500);
        ringout(robot, 0.52);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0);

        // Collecting the ring on the mat and shoot it
        robot.intake.setPower(1);
        strafe(robot, -15, -40, 90);
        robot.intake.setPower(0);

        // Reposition the robot to the shooting position
        ringout(robot, 0.5);
        strafe(robot, -13, -60, 90);
        ringshot(robot);
        ringout(robot, 0);
        sleep(500);
        robot.intake.setPower(0);

        strafe(robot, -44, -20, 270);
        strafe(robot,-44, -11, 270);
        sleep(750);
        robot.wobbleGrab.setPosition(1);
        sleep(500);
        robot.wobbleSet.setPower(-0.7);
        sleep(1000);
        robot.wobbleSet.setPower(0);

        strafe(robot, -25, -20, 270);
        strafe(robot, 0, -100, 90);
        robot.wobbleSet.setPower(1);
        sleep(500);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        robot.wobbleGrab.setPosition(0);
        sleep(500);
        strafe(robot, 0, -75, 0);

    }

    private void C(DiscoDrive robot){
        // Driving to Zone A
        robot.ringArm.setPower(1.0);
        robot.turn(Math.toRadians(90));
        strafe(robot, -0, -50, 90);
        strafe(robot, -30, -115, 90);
        robot.wobbleSet.setPower(1);
        sleep(200);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        robot.wobbleGrab.setPosition(1);
        sleep(500);
        robot.wobbleGrab.setPosition(0);
        sleep(500);

        // Reposition to the shooting position
        strafe(robot, -15, -60, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,-0.3,500);
        ringout(robot, 0.52);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0);

        // Collecting the ring on the mat and shoot it
        robot.intake.setPower(1);
        strafe(robot, -15, -30, 90);
        robot.intake.setPower(0);

        // Reposition the robot to the shooting position
        strafe(robot, -15, -60, 90);
        ringreverse(robot,-0.3,500);
        ringout(robot, 0.52);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0);


        // Park on the line
        strafe(robot, -13, -70, 90);
    }

    private void goForward(DiscoDrive robot, double distance) {
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .forward(distance)
                .build();
        robot.followTrajectory(move);
    }

    private void strafe(DiscoDrive robot, double x, double y, double heading) {
        Trajectory strafeLeft = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
                .build();
        robot.followTrajectory(strafeLeft);
    }

    private void splineTo(DiscoDrive robot, double x, double y, double heading) {
        Trajectory spline = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)), 0)
                .build();
        robot.followTrajectory(spline);
    }

    private void ringout(DiscoDrive robot, double power) {
        robot.shooterL.setPower(power);
        robot.shooterR.setPower(-power);
    }

    private void ringreverse(DiscoDrive robot, double power, long time) {
        robot.shooterL.setPower(-power);
        robot.shooterR.setPower(power);
        sleep(time);
    }

    private void ringshot(DiscoDrive driveTrain) {
        driveTrain.intake.setPower(0);
        sleep(750);
        driveTrain.intake.setPower(1);
        sleep(125);
    }
}
