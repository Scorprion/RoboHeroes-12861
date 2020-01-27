package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesBs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesBJustPark", group = "Hermes")
public class HermesBJustPark extends HermesAggregated {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(-0.2, 90, 4, 4);
        encoderDrives(0.4, -39, -39, 5);


    }
}