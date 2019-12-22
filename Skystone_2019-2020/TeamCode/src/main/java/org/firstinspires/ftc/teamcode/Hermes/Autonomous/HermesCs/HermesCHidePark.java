package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesCHidePark", group = "Hermes")
public class HermesCHidePark extends HermesAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(0.2, 90, 4, 4);
        encoderDrives(0.6, 28, 28, 5);
        sleep(20000);
        encoderDrives(0.6, -66, -66 , 7);
    }
}