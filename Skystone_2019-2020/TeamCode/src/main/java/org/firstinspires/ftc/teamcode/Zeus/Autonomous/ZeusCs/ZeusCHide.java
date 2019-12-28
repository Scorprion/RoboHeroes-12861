package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusCHide", group = "Zeus")
public class ZeusCHide extends ZeusAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.StoneLift.setPosition(0.6);
        mecanumMove(0.2, 90, 4, 3);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.6, 28, 28, 5);
    }
}
