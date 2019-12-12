package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesMecTest", group = "Hermes")
public class HermesMecTest extends HermesAggregated {
    public boolean VuforiaFound = false;
    double last_error = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        /*while(opModeIsActive() && !closeEnoughTo(last_error, 1, 0)) {
            last_error = pidDynamic((likeallelse(robot.imu.getAngularOrientation().firstAngle)), last_error,1/10,
                    1.5, 0.5, 0, 90, 0.35, true);
        }*/
        pidTurn(3, 1.2, 0, 90, 0.25, 4);
        telemetry.addData("Angle: ", likeallelse(robot.imu.getAngularOrientation().firstAngle));
        telemetry.update();
        sleep(10000);

        // mecanumMove(0.3, 90, 24, 3);

    }
}