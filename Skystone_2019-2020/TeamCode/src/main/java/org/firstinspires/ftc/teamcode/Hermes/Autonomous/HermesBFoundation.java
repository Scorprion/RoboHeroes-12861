package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesBFoundation", group = "Hermes")

public class HermesBFoundation extends HermesAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(0.5,90,12,5);
        sleep(500);
        encoderDrives(0.5,24,24,6);
        sleep(500);
        encoderDrives(0.1, 10, 10,5);
        sleep(500);
        robot.FoundationClaw.setPower(-1);
        sleep(500);
        mecanumMove(-0.5, 90, 16, 5);
        sleep(500);
        encoderDrives(0.4, -15, -15, 5);
        sleep(500);
        encoderDrives(0.4, -26, 26, 5);
        sleep(500);
        encoderDrives(0.5,18,18,10);
    }
}