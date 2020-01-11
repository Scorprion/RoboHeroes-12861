package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusBs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusBHidePark", group = "Hermes")
public class ZeusBHidePark extends ZeusAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(-0.2, 90, 4, 4);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.6, 28, 28, 5);
        robot.StoneLift.setPosition(0.6);
        sleep(20000);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.6, -66, -66 , 7);
    }
}