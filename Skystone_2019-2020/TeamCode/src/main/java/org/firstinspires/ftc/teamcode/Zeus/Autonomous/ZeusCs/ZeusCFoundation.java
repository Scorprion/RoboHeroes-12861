package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusCFoundation", group = "Zeus")
public class ZeusCFoundation extends ZeusAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.5, 24, 24, 1.5);
        mecanumMove(-0.5, 90, 12, 1.5);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.25, 13, 13, 2.5);

        robot.FoundationClaw.setPower(1);  // Don't kill anyone
        sleep(1750);
        robot.FoundationClaw.setPower(0);
        sleep(1000);

        mecanumMove(0.5, 90, 12, 1.7);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.5, -30, -30, 1.5);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.4, 45, -45, 2.5);
        robot.StoneLift.setPosition(0.6);// Turning 90 degrees
        encoderDrives(0.4, 18, 18, 1.5);

        robot.FoundationClaw.setPower(-1);  // Don't kill anyone
        sleep(1000);
        robot.FoundationClaw.setPower(0);
        encoderDrives(0.4, -12, -12, 1.5);
    }
}
