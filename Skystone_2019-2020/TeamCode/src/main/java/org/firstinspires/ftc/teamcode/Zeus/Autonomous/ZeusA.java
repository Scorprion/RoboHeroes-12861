package org.firstinspires.ftc.teamcode.Zeus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusA", group = "Zeus")
public class ZeusA extends ZeusAggregated {
    private double speed = 0.1, pidOutput = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        /*

        //init_vuforia();

        waitForStart();

        mecanumMove(0.4, 45, 11, 2);
        sleep(500);
        // start_vuforia();

        */

        init_vuforia();

        waitForStart();

        mecanumMove(-0.3, 90, 100, 2.5);
        pidTurn(1.22, 0.5, 0.1, 0, 0, 1.5);
        sleep(500);
        start_vuforia();
        sleep(500);
        encoderDrives(0.3, 4, 4, 1);
        sleep(250);
        mecanumMove(-0.3, 90, 16, 4);
        sleep(500);

        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        encoderDrives(0.6, -60, -60, 10);


        /*
        sleep(500);
        encoderDrives(0.6, 27, 27, 5);
        */

    }
}