package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesD", group = "Hermes")
public class HermesD extends HermesAggregated {
    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private double locationV = -1000;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        init_vuforia();

        waitForStart();

        isD = true;

        mecanumMove(-0.3, 90, 19, 3);
        sleep(500);
        start_vuforia();
        sleep(100);
        encoderDrives(0.2, 4, 4, 1);
        sleep(500);
        mecanumMove(-0.3, 90, 16, 4);
        sleep(500);
        robot.Gate.setPower(-0.5);
        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        encoderDrives(0.6, -60, -60, 10);
        sleep(500);
        robot.Gate.setPower(1);
        sleep(750);
        encoderDrives(0.6, 24, 24, 5);
        sleep(500);
        mecanumMove(-0.4, 90, 12, 3);
    }
}