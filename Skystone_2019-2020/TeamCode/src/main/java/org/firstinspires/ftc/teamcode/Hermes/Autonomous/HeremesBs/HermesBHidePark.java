package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HeremesBs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesBHidePark", group = "Hermes")
public class HermesBHidePark extends HermesAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(-0.2, 90, 4, 4);
        encoderDrives(0.6, 28, 28, 5);
        sleep(20000);
        encoderDrives(0.6, -66, -66 , 7);
    }
}