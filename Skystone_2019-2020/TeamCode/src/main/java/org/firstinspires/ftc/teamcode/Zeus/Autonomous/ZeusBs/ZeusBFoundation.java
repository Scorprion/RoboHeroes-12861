package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusBs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusBFoundation", group = "Hermes")
public class ZeusBFoundation extends ZeusAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        encoderDrives(0.5, 24, 24, 1.5);
        robot.StoneLift.setPosition(0.6);
        mecanumMove(0.5, 90, 12, 1.5);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.25, 10, 10, 2.5);
        robot.StoneLift.setPosition(0.6);

        robot.FoundationClaw.setPower(1);
        robot.StoneLift.setPosition(0.6);
        sleep(1750);
        robot.FoundationClaw.setPower(0);
        robot.StoneLift.setPosition(0.6);
        sleep(1000);

        //mecanumMove(-0.5, 90, 12, 1.7);
        encoderDrives(0.5, -15, -15, 1.5);
        robot.StoneLift.setPosition(0.6);
        pidTurn(2.5, 0.2, 0.1, -90, 0, 3.0);
        robot.StoneLift.setPosition(0.6);
        //encoderDrives(0.4, 45, -45, 2.5); // Turning 90 degrees
        encoderDrives(0.4, 20, 24, 1.5);
        robot.StoneLift.setPosition(0.6);

        robot.FoundationClaw.setPower(-1);
        robot.StoneLift.setPosition(0.6);
        sleep(1000);
        robot.FoundationClaw.setPower(0);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.5, -47, -47, 5);
        robot.StoneLift.setPosition(0.6);
        /*mecanumMove(0.5, 90, 10, 1.5);
        encoderDrives(0.4, -24, -24, 1.5);*/
    }
}