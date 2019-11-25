package org.firstinspires.ftc.teamcode.Zeus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusA", group = "Hermes")
public class ZeusA extends ZeusAggregated {
    private double speed = 0.1, pidOutput = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //init_vuforia();

        waitForStart();

        mecanumMove(0.4, 45, 11, 2);
        sleep(500);
        // start_vuforia();
    }
}