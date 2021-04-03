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
        strafe(driveTrain, 1, -33, 0);

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
        if (minDistance >= 5) {  // None
            telemetry.addLine("NONE");
            A(driveTrain);
        } else if (minDistance >= 3.3) {  // SINGLE
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
        sleep(750);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        wobbleoutput(robot,1200);

        // Reposition to the shooting position
        strafe(robot, -13, -58, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,0.5,500);
        ringout(robot, 0.49);
        sleep(750);
        ringshot(robot);
        sleep(200);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);

        ringout(robot, 0);
        strafe(robot,-5, -17.5, 90);
        strafe(robot,-12,-17.5,90);
        wobbleintake(robot,1400);

        strafe(robot, -25, -75, 90);

        // Dropping the wobble goal
        wobbleoutput(robot,1400);
        strafe(robot, -20, -75, 90);
        robot.ringArm.setPower(-1.0);
        strafe(robot, -13, -56, 90);


    }

    private void B(DiscoDrive robot) {
        // Driving to Zone B
        robot.ringArm.setPower(1.0);
        robot.turn(Math.toRadians(90));
        strafe(robot, -3, -95, 90);
        robot.wobbleSet.setPower(1);
        sleep(750);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        wobbleoutput(robot,1400);

        // Reposition to the shooting position
        strafe(robot, -3, -56, 90);
        strafe(robot, -13, -56, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,0.5,500);
        ringout(robot, 0.5);
        sleep(500);
        ringshot(robot);
        ringshot(robot);
        sleep(200);
        ringshot(robot);
        sleep(500);
        ringout(robot, 0);

        robot.shooterL.setPower(-0.5);
        robot.shooterR.setPower(-0.5);
        strafe(robot,-5, -18.5, 90);
        strafe(robot,-13,-18.5,90);
        robot.intake.setPower(0);
        robot.shooterL.setPower(-0);
        robot.shooterR.setPower(0);
        wobbleintake(robot,1400);


        strafe(robot, -3, -90, 90);

        // Dropping the wobble goal
        wobbleoutput(robot,1200);
        strafe(robot, 10, -90,90);
        robot.ringArm.setPower(-1.0);
        ringout(robot, 0.5);
        strafe(robot, -13, -56, 90);
        ringshot(robot);    

    }

    private void C(DiscoDrive robot){
        // Driving to Zone C
        robot.turn(Math.toRadians(90));
        robot.ringArm.setPower(1.0);
        strafe(robot, -0, -50, 90);
        strafe(robot, -30, -115, 90);
        robot.wobbleSet.setPower(1);
        wobbleoutput(robot,1400);
        robot.wobbleSet.setPower(0);

        // Reposition to the shooting position
        strafe(robot, -13, -56, 90);

        // Shooting the preloaded rings out
        ringreverse(robot,1,500);
        ringout(robot, 0.5);
        sleep(500);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0);

        // Collecting the ring on the mat and shoot it
        strafe(robot, -13, -45, 90);
        sleep(250);
        robot.shooterL.setPower(-0.3);
        robot.shooterR.setPower(-0.3);
        robot.intake.setPower(1);
        strafe(robot,-11,-5,90);
        sleep(100);
        robot.intake.setPower(0);
        robot.shooterL.setPower(0);
        robot.shooterR.setPower(0);

        // Reposition the robot to the shooting position
        robot.ringArm.setPower(-1.0);
        strafe(robot, -13, -54, 90);
        ringreverse(robot,0.5,500);
        ringout(robot, 0.49);
        sleep(500);
        ringshot(robot);
        ringshot(robot);
        sleep(200);
        ringshot(robot);
        sleep(2000);
        robot.intake.setPower(0);
        ringout(robot, 0);


        // Park on the line
        strafe(robot, -13, -74, 90);
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
        robot.shooterR.setPower(power);
    }

    private void ringreverse(DiscoDrive robot, double power, long time) {
        robot.shooterL.setPower(-power);
        robot.shooterR.setPower(-power);
        sleep(time);
    }

    private void ringshot(DiscoDrive robot) {
        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(200);
    }

    private void wobbleintake(DiscoDrive robot, long time){
        robot.WobbleGrabL.setPower(-1);
        robot.WobbleGrabR.setPower(1);
        sleep(time);
        robot.WobbleGrabL.setPower(0);
        robot.WobbleGrabR.setPower(0);
    }

    private void wobbleoutput(DiscoDrive robot, long time){
        robot.WobbleGrabL.setPower(1);
        robot.WobbleGrabR.setPower(-1);
        sleep(time);
        robot.WobbleGrabL.setPower(0);
        robot.WobbleGrabR.setPower(0);
    }
}
