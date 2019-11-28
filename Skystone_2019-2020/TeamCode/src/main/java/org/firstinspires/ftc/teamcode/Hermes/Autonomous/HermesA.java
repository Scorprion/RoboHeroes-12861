package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Hermes")
public class HermesA extends HermesAggregated {
    private double speed = 0.1, pidOutput = 0;
    private VectorF locationV;
    public boolean VuforiaFound = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        init_vuforia();

        waitForStart();

        mecanumMove(0.3, 90, 10.5, 2.5);
        sleep(500);
        start_vuforia();
        sleep(500);
        mecanumMove(0.4, 90, 24.5, 3.5);
        sleep(500);
        robot.Gate.setPower(-0.5);
        sleep(500);
        mecanumMove(0.3, -90, 13, 3);
    }
}