package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Config
@Autonomous(name = "PIDTune", group = "Hermes")
public class PIDTune extends HermesAggregated {
    public static double P = 1;
    public static double I = 0;
    public static double D = 0;
    public static double setpoint = 0;
    public static double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        PID pid = new PID(P, I, D, 0.3);

        double out;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pid.P = P;
            pid.I = I;
            pid.D = D;
            // Manual error updating so h
            // Normalizes the output between 0.2 and 1 (since the robot won't even move if it's below 0.2 power)
            // out = pid.getPID(0.2 + ((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) * 0.8) / 360);
            out = pid.constrain(pid.getPID((setpoint - pid.likeallelse(robot.imu.getAngularOrientation().firstAngle)) / 36), -Math.abs(speed - 1), Math.abs(1 - speed));
            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Angle", pid.total_angle);
            telemetry.addData("Error", pid.error);
            telemetry.addData("Out", out);
            telemetry.addData("Pout", pid.Poutput);
            telemetry.addData("Iout", pid.Ioutput);
            telemetry.addData("Dout", pid.Doutput);
            telemetry.addData("P", pid.P);
            telemetry.addData("I", pid.I);
            telemetry.addData("D", pid.D);
            telemetry.addData("Speed", speed + out);

            robot.FrontRight.setPower(-speed - out);
            robot.BackRight.setPower(-speed - out);
            robot.FrontLeft.setPower(speed + out);
            robot.BackLeft.setPower(speed + out);
            telemetry.update();
        }
    }
}