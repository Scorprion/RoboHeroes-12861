package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesCFoundation", group = "Hermes")
@Disabled

public class HermesCFoundation extends HermesAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        encoderDrives(0.5, 24, 24, 1.5);
        mecanumMove(-0.5, 90, 13, 1.5);
        encoderDrives(0.25, 10, 10, 2.5);

        robot.FoundationClaw.setPower(1);
        sleep(1750);
        robot.FoundationClaw.setPower(0);
        sleep(1000);

        //mecanumMove(0.5, 90, 12, 1.7);
        encoderDrives(0.5, -15, -15, 1.5);
        pidTurn(2.0, 0.5, 0.08, 90, 0, 3.0);
        //encoderDrives(0.4, 45, -45, 2.5); // Turning 90 degrees
        encoderDrives(0.4, 24, 20, 1.5);

        robot.FoundationClaw.setPower(-1);
        sleep(1000);
        robot.FoundationClaw.setPower(0);
        encoderDrives(0.5, -47, -47, 5);
        mecanumMove(0.5, 90, 5, 1.5);
    }
}