package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

import java.io.IOException;

@Autonomous(name = "IMUTest", group = "Hermes")
public class IMUTest extends HermesAggregated {
    private ElapsedTime timer = new ElapsedTime();
    double total_angle = 0, last_angle = 0, current_angle = 0, delta_angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            current_angle = robot.imu.getAngularOrientation().firstAngle;
            delta_angle = current_angle - last_angle;
            total_angle += likeallelse(delta_angle);
            telemetry.addData("Total angle", total_angle);
            telemetry.addData("Angular Velocity", robot.imu.getAngularVelocity().xRotationRate);
            telemetry.update();

            last_angle = current_angle;
        }
    }
}