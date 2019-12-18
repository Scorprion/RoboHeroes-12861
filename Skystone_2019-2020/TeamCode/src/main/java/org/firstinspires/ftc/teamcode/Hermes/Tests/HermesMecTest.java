package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesMecTest", group = "Hermes")
public class HermesMecTest extends HermesAggregated {
    public boolean VuforiaFound = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        //mecanumMove(0.3, -90, 24, 3);
        /*while(opModeIsActive() && !closeEnoughTo(last_error, 1, 0)) {
            last_error = pidDynamic((likeallelse(robot.imu.getAngularOrientation().firstAngle)), last_error,1/10,
                    1.5, 0.5, 0, 90, 0.35, true);
        }*/
        pidTurn(0.66, 1.33, 0.33, 90, 0.25, 400);
        telemetry.addData("Angle: ", likeallelse(robot.imu.getAngularOrientation().firstAngle));
        telemetry.update();

    }
}