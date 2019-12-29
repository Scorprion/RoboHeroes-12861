package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

import java.io.IOException;

@Autonomous(name = "CSVData", group = "Hermes")
public class CSVData extends HermesAggregated {
    private ElapsedTime timer = new ElapsedTime();
    double speed = 0.1;
    double counter = 3;
    double average = 0;
    double last_avg = 0, enocder_veloc;
    double last_time = 0, delta_time;
    double op_time = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        DataLogger d = null;
        try {
            d = new DataLogger("test.csv");
            d.addHeaderLine("Time,", "Input,", "Angle,", "Velocity,", "Encoder,", "E_Velocity");
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            op_time = timer.seconds();

            if(op_time > counter) {
                counter += 3;
                speed += 0.1;
            }

            robot.BackLeft.setPower(speed);
            robot.FrontLeft.setPower(speed);
            robot.FrontRight.setPower(-speed);
            robot.BackRight.setPower(-speed);


            average = (robot.FrontLeft.getCurrentPosition() +
                    robot.BackLeft.getCurrentPosition()) / 2;
            delta_time = last_time - op_time;

            enocder_veloc = (last_avg - average) / delta_time;
            telemetry.addData("Time", timer.milliseconds() / 1000);
            telemetry.addData("Input", speed);
            telemetry.addData("Angle", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Velocity", robot.imu.getAngularVelocity().xRotationRate);
            telemetry.addData("Encoder", average);
            telemetry.addData("E_Velocity", enocder_veloc);
            telemetry.update();

            try {
                d.addDataLine(time + "," + speed + "," +
                        robot.imu.getAngularOrientation().firstAngle + "," +
                        robot.imu.getAngularVelocity().xRotationRate + "," +
                        average + "," +
                        enocder_veloc);
            } catch (IOException e) {
                e.printStackTrace();
            }

            last_avg = average;
            last_time = op_time;
            // Manual error updating so h
            // Normalizes the output between 0.2 and 1 (since the robot won't even move if it's below 0.2 power)
            // out = pid.getPID(0.2 + ((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) * 0.8) / 360);

        }
    }
}