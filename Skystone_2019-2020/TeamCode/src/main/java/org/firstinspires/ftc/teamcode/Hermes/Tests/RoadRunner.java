package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Config
@Autonomous(name = "RoadRunner", group = "Hermes")
public class RoadRunner extends HermesAggregated {
    public static double P = 2.0, I = 0.5, D = 0.08, setpoint = 0, target_veloc = 10,
            correction = 0, max_speed = 1, speed = 0.8, angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        PIDCoefficients coeff = new PIDCoefficients(P, I, D);
        PIDFController controller = new PIDFController(coeff);
        controller.setTargetVelocity(target_veloc);
        controller.setTargetPosition(setpoint);
        controller.setOutputBounds(0, max_speed - speed);
        // controller.setInputBounds(0.0, 2.0 * Math.PI);

        waitForStart();

        while(opModeIsActive()) {
            angle = likeallelse(robot.imu.getAngularOrientation().firstAngle);
            correction = controller.update(angle);

            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Angle", angle);
            telemetry.addData("Error", controller.getLastError());
            telemetry.addData("Correction", correction);
            telemetry.update();

            robot.FrontRight.setPower(-speed - correction);
            robot.BackRight.setPower(-speed - correction);
            robot.FrontLeft.setPower(speed + correction);
            robot.BackLeft.setPower(speed + correction);
        }
    }
}

