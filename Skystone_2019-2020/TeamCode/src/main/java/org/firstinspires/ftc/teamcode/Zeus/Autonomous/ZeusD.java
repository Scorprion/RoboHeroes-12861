package org.firstinspires.ftc.teamcode.Zeus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusD", group = "Zeus")
public class ZeusD extends ZeusAggregated {
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
        //Compensation implemented
        mecanumMove(-0.3, 90, 100, 2.5);

        sleep(500);
        //start_vuforia();
        sleep(500);
        //pidTurn(2.45, 0.1,  0.1, 90, 0, 1.5);
        sleep(500);

        //mecanumMove(-0.3, 90, 100, 2.5);
        sleep(500);
        encoderDrives(0.6, -60, -60, 10);


        /*
        sleep(500);
        encoderDrives(0.6, 27, 27, 5);
        */

    }
}