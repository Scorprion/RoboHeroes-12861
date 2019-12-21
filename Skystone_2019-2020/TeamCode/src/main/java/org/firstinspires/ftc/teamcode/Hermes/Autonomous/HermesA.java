package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Hermes")
public class HermesA extends HermesAggregated {
    public boolean VuforiaFound = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //init_vuforia();

        waitForStart();

        mecanumMove(-0.3, 90, 100, 3);
        /*pidTurn(1.22, 0.5, 0.1, 0, 0, 1.5);
        sleep(500);
        start_vuforia();
        sleep(500);
        encoderDrives(0.3, 4, 4, 1);
        sleep(250);
        mecanumMove(-0.3, 90, 16, 4);
        sleep(500);
        robot.Gate.setPower(-0.4);
        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        encoderDrives(0.6, -60, -60, 10);
        sleep(500);
        robot.Gate.setPower(0.4);
        sleep(500);
        encoderDrives(0.6, 27, 27, 5);*/

    }
}