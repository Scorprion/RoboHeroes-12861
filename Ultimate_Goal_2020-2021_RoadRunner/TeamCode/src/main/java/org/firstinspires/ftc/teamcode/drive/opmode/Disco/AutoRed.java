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
        driveTrain.ringArm.setPosition(0.1);
        // Move towards rings
        strafe(driveTrain, 1, -32.5, 0);

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
        if (minDistance >= 4.7) {  // None
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
        robot.ringArm.setPosition(1);
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
        ringout(robot, 0.5);
        sleep(700);

        robot.intake.setPower(0);
        sleep(900);
        ringshot(robot);

        robot.intake.setPower(0);
        sleep(1200);
        robot.intake.setPower(1);
        sleep(600);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(1200);

        ringout(robot, 0);
        robot.intake.setPower(0);

        strafe(robot,-5, -18.5, 90);
        strafe(robot,-10,-18.5,90);
        wobbleintake(robot,1400);

        strafe(robot, -25, -75, 90);

        // Dropping the wobble goal
        wobbleoutput(robot,1400);
        strafe(robot, -20, -75, 90);
        robot.ringArm.setPosition(0.1);
        strafe(robot, -13, -62, 90);


    }

    private void B(DiscoDrive robot) {
        // Driving to Zone B
        robot.ringArm.setPosition(1);
        robot.turn(Math.toRadians(90));
        strafe(robot, -3, -95, 90);
        robot.wobbleSet.setPower(1);
        sleep(750);
        robot.wobbleSet.setPower(0);

        // Dropping the wobble goal
        wobbleoutput(robot,1400);

        // Reposition to the shooting position
        strafe(robot, 2, -95, 90);
        strafe(robot, -13, -56, 90);

        // Shooting the preloaded rings out
        ringout(robot, 0.5);
        sleep(700);

        robot.intake.setPower(0);
        sleep(900);
        ringshot(robot);

        robot.intake.setPower(0);
        sleep(1200);
        robot.intake.setPower(1);
        sleep(600);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(1200);

        ringout(robot, 0);
        robot.intake.setPower(1);

        robot.shooterL.setPower(-0.5);
        robot.shooterR.setPower(-0.5);
        strafe(robot,-5, -18.5, 90);
        strafe(robot,-11.5,-18.5,90);
        robot.intake.setPower(0);
        robot.shooterL.setPower(-0);
        robot.shooterR.setPower(0);
        wobbleintake(robot,1400);

        strafe(robot, -3, -92, 90);

        // Dropping the wobble goal
        wobbleoutput(robot,1200);
        ringout(robot, -0.5);
        robot.intake.setPower(-0.35);
        ringout(robot, 0.45);
        strafe(robot, 10, -62, 90);
        robot.intake.setPower(0);
        robot.ringArm.setPosition(0.1);
        sleep(1000);
        robot.intake.setPower(1);
        sleep(2000);

    }

    private void C(DiscoDrive robot){
        // Driving to Zone C
        robot.turn(Math.toRadians(90));
        robot.ringArm.setPosition(1);
        strafe(robot, -0, -50, 90);
        strafe(robot, -30, -115, 90);
        robot.wobbleSet.setPower(1);
        wobbleoutput(robot,1400);
        robot.wobbleSet.setPower(0);

        // Reposition to the shooting position
        strafe(robot, -13, -60, 90);

        // Shooting the preloaded rings out
        ringout(robot, 0.51);
        sleep(1000);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(400);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(900);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(1500);
        ringout(robot, 0);
        robot.intake.setPower(0);

        // Collecting the ring on the mat and shoot it
        robot.intake.setPower(1);
        robot.shooterL.setPower(-1);
        robot.shooterR.setPower(-1);
        strafe(robot,-11,-25,90);
        robot.intake.setPower(0);
        robot.shooterL.setPower(0);
        robot.shooterR.setPower(0);

        // Reposition the robot to the shooting position
        robot.ringArm.setPosition(0.1);
        strafe(robot, -13, -56, 90);
        ringreverse(robot,0.5,500);
        sleep(500);
        robot.intake.setPower(-0.5);
        sleep(500);
        robot.intake.setPower(0);
        ringout(robot, 0.49);
        sleep(1000);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(150);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(400);

        robot.intake.setPower(0);
        sleep(900);
        robot.intake.setPower(1);
        sleep(900);
        robot.intake.setPower(0);
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
        sleep(150);
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
