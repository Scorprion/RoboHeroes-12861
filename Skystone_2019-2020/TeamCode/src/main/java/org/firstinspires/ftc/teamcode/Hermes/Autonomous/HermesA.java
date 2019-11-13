package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Autonomous")
public class HermesA extends HermesAggregated {
    private double speed = 0.1, pidOutput = 0;
    private VectorF locationV;
    public boolean VuforiaFound = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        mecanumMove(0.5, 45, 10, 1);
        mecanumMove(0.5, 270, 10, 1);
        mecanumMove(-0.5, 0, 10, 1);
    }
}