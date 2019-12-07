package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Hermes")
public class HermesA extends HermesAggregated {
    public boolean VuforiaFound = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        init_vuforia();

        waitForStart();

        //MecTime(-1, 1, 1, -1,500);
        mecanumMove(-0.3, 90, 17, 3);
        sleep(500);
        start_vuforia();
        sleep(100);
        encoderDrives(0.2, -5, -5, 1);
        sleep(500);
        mecanumMove(-0.3, 90, 14, 4);
        sleep(500);
        robot.Gate.setPower(-1);
        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        encoderDrives(0.6, 60, 60, 10);
        sleep(500);
        robot.Gate.setPower(0.7);
        sleep(500);
        encoderDrives(0.6, -23, -23, 5);
        sleep(500);
        mecanumMove(-0.4, 90, 12, 3);

    }
}