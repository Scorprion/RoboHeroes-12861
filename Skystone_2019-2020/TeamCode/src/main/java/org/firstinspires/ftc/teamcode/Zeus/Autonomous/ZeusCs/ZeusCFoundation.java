package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        sleep(200);
        mecanumMove(0.3, 270, 20, 2);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.25, 11, 12, 4);

        robot.FoundationClaw.setPower(1);  // Don't kill anyone
        sleep(1750);
        robot.FoundationClaw.setPower(1);
        sleep(1000);

        robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(100);
        mecanumMove(0.5,270,-15,4);
        sleep(100);

        encoderDrives(0.5,-6,6,5);
        sleep(100);
        encoderDrives(0.5,-17,-17,5);
        //encoderDrives(0.5,-30,-20,5);
        sleep(200);
        encoderDrives(0.5,-18,18,5);
        sleep(200);

        //encoderDrives(1,-120,20,10);
        robot.StoneLift.setPosition(0.6);
        //encoderDrives(0.5, 30, 30, 1.5);
        robot.StoneLift.setPosition(0.6);

        robot.FoundationClaw.setPower(-1);  // Don't kill anyone
        sleep(1000);
        robot.FoundationClaw.setPower(0);

        encoderDrives(0.4, -12, -12, 1.5);
    }
}
