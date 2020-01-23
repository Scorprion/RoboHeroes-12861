package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesMecTest", group = "Hermes")
public class HermesMecTest extends HermesAggregated {

    PID tester = new PID(2, 0.5, 0.08, 0.3);
    double error_factor = (1.0 / 36.0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            //pidDynamic(likeallelse(robot.imu.getAngularOrientation().firstAngle), error_factor,
                    //1, 0, 0.04, 0, 0.25, -0.25, direction.STRAFE);
            pidDynamic(likeallelse(robot.imu.getAngularOrientation().firstAngle), error_factor,
            2, 0.5, 0.08, 0, 0.25, -0.25, direction.STRAFE);
            telemetry.addData("Angle: ", tester.likeallelse(robot.imu.getAngularOrientation().firstAngle));
            telemetry.update();
        }

        // mecanumMove(-0.4, 90, 31, 5);
    }
}