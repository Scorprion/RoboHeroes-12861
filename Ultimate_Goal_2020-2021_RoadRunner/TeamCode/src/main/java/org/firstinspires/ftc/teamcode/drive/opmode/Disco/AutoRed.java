package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DiscoDrive;

@Autonomous(group="Disco")
public class AutoRed extends LinearOpMode {
    double distance;

    Double minDistance = Double.POSITIVE_INFINITY;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DiscoDrive driveTrain = new DiscoDrive(hardwareMap);

        waitForStart();

        // Lowers the distance sensor
        driveTrain.ringArm.setPower(-1.0);
        // Move towards rings
        strafe(driveTrain, 0, -30);

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

        // Raises the distance sensor
        driveTrain.ringArm.setPower(1.0);
        sleep(1250);
        driveTrain.ringArm.setPower(0);

        // Scans the rings for the wobble goal placement
        if (minDistance >= 4.5) {  // None
            telemetry.addLine("NONE");
            A(driveTrain);
        } else if (minDistance >= 4) {  // SINGLE
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
        robot.turn(Math.toRadians(90));
        strafe(robot, -40, -100);

        // Dropping the wobble goal
        robot.wobbleSet.setPower(0.5);
        sleep(900);
        robot.wobbleGrab.setPosition(0);
        sleep(1000);
        robot.wobbleGrab.setPosition(1);
        sleep(1000);
        goForward(robot, 15);
        robot.wobbleSet.setPower(-0.4);
        sleep(700);

        // Grab the 2nd Wobble Goal
    }

    private void B(DiscoDrive robot){
        // Driving to Zone B
        robot.turn(Math.toRadians(90));
        strafe(robot, 10, -150);

        // Dropping the wobble goal
        robot.wobbleSet.setPower(0.5);
        sleep(900);
        robot.wobbleGrab.setPosition(0);
        sleep(1000);
        robot.wobbleGrab.setPosition(1);
        sleep(1000);
        goForward(robot, 15);
        robot.wobbleSet.setPower(-0.4);
        sleep(700);

        // Head over to the ring on the map

        strafe(robot,0,-60);

        // Collection and shooting of the 4 rings on the map
        robot.intake.setPower(-1);
        goForward(robot, 20);
        robot.intake.setPower(1);
        goForward(robot, 22);
        goForward(robot, -45);
        ringshot(robot);
    }

    private void C(DiscoDrive robot){
        // Driving to Zone A
        robot.turn(Math.toRadians(90));
        strafe(robot, 40, -125);

        // Dropping the wobble goal
        robot.wobbleSet.setPower(0.5);
        sleep(900);
        robot.wobbleGrab.setPosition(0);
        sleep(1000);
        robot.wobbleGrab.setPosition(1);
        sleep(1000);
        goForward(robot, 15);
        robot.wobbleSet.setPower(-0.4);
        sleep(700);

        // Head over to the ring on the map
        strafe(robot, 10, -41);

        // Collection and shooting of the 4 rings on the map
        robot.intake.setPower(-1);
        goForward(robot, 20);
        robot.intake.setPower(1);
        goForward(robot, 22);
        goForward(robot, -45);
        ringshot(robot);
        ringshot(robot);
        ringshot(robot);
    }

    private void goForward(DiscoDrive robot, double distance) {
        Trajectory move = robot.trajectoryBuilder(robot.getPoseEstimate())
                .forward(distance)
                .build();
        robot.followTrajectory(move);
    }

    private void strafe(DiscoDrive robot, double x, double y) {
        Trajectory strafeLeft = robot.trajectoryBuilder(robot.getPoseEstimate())
                .strafeTo(new Vector2d(x, y))
                .build();
        robot.followTrajectory(strafeLeft);
    }

    private void ringout(DiscoDrive robot, double power, long time) {
        robot.shooterL.setPower(power);
        robot.shooterR.setPower(-power);
        sleep(time);
    }

    private void ringreverse(DiscoDrive robot, double power, long time) {
        robot.shooterL.setPower(-power);
        robot.shooterR.setPower(power);
        sleep(time);
    }

    private void ringshot(DiscoDrive driveTrain) {
        driveTrain.intake.setPower(0);
        ringreverse(driveTrain, 0.4, 800);
        ringout(driveTrain, 0.7, 1000);
        driveTrain.intake.setPower(1);
        sleep(2000);
    }
}
