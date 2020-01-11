package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusBs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusBJustPark", group = "Hermes")
public class ZeusBJustPark extends ZeusAggregated {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(-0.2, 90, 4, 4);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.4, -39, -39, 5);


    }
}