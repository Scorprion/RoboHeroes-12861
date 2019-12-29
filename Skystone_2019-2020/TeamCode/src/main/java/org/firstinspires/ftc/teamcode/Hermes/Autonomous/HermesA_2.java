package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA_2", group = "Hermes")
public class HermesA_2 extends HermesAggregated {
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        init_vuforia();

        waitForStart();

        mecanumMove(-0.3, 90, 17, 2.5);
        pidTurn(2.5, 0.2, 0.1, 0, 0, 1.5);
        sleep(250);
        start_vuforia();
        sleep(250);
        robot.Gate.setPower(-0.4);
        sleep(250);
        mecanumMove(-0.3, 90, 12, 4);
        sleep(250);
        robot.Clamper.setPower(-1);
        sleep(1500);
        robot.Gate.setPower(0.5);
        sleep(250);
        mecanumMove(0.5, 90, 6, 5);
        sleep(250);
        mecanumMove(-0.6, 90, 40, 3);
        sleep(250);
        pidTurn(2.5, 0.2, 0.1, 90, 0, 1.5);
        sleep(250);
        encoderDrives(0.6, -40, -40, 10);
        sleep(250);
        robot.Gate.setPower(-0.4);
        sleep(250);
        robot.Clamper.setPower(1);
        sleep(750);
        robot.Gate.setPower(0.4);
        pidTurn(2.5, 0.2, 0.1, 0, 0, 2);
        sleep(250);
        encoderDrives(0.6, 40, 40, 5);

    }
}
