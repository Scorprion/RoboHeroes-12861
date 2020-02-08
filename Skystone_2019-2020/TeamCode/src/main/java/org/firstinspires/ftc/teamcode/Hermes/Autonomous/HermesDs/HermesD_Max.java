package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesDs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesD_Max", group = "Hermes")
public class HermesD_Max extends HermesAggregated {
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //init_vuforia();

        waitForStart();

        encoderDrives(1, 23,23,5);
        mecanumMove(-0.6, 90, 30, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor(true);
            telemetry.addLine("Checking position");
            telemetry.update();
        }

        if(pos == position.WALL) {
            //First SkyStone
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 5, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.85, -108, -108, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.8, 90, 12, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(1, 90, 8, 3);

            //Second SkyStone
            encoderDrives(0.8, 86, 86, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.5, 90, 7, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            encoderDrives(0.85, -93, -93, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.7, -90, 8, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(100);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(1, 90, 7, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            mecanumMove(0.5, -90, 12, 5);
            sleep(100);
            mecanumMove(0.5, 90, 8, 5);
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            encoderDrives(0.4, 23, 23, 5); //Drive forward to clamp the foundation
            encoderDrives(1, -55, -50, 5); //Drive back with the foundation
            robot.FoundationClaw.setPower(-1);
            sleep(500);
            mecanumMove(1, 100, 50, 5);
            robot.FoundationClaw.setPower(0);
            telemetry.addLine("FIRST");

        } else if(pos == position.MIDDLE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            encoderDrives(0.6, -8, -8,5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 5, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.9, -100, -100, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.8, 90, 12, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(1, 90, 8, 3);

            //Second SkyStone
            encoderDrives(0.8, 78, 78, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.5, 90, 7, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            encoderDrives(0.85, -93, -93, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.7, -90, 17, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(100);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(1, 90, 7, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            mecanumMove(0.5, -90, 12, 5);
            sleep(100);
            mecanumMove(0.5, 90, 8, 5);
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            encoderDrives(0.4, 23, 23, 5); //Drive forward to clamp the foundation
            encoderDrives(1, -55, -50, 5); //Drive back with the foundation
            robot.FoundationClaw.setPower(-1);
            sleep(500);
            mecanumMove(1, 100, 50, 5);
            robot.FoundationClaw.setPower(0);
            telemetry.addLine("SECOND");

        } else if(pos == position.BRIDGE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            encoderDrives(0.6, -16, -16,5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 5, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.85, -92, -92, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.8, 90, 12, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(1, 90, 8, 3);

            //Second SkyStone
            encoderDrives(0.8, 70, 70, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.5, 90, 7, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            encoderDrives(0.85, -85, -85, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.7, -90, 17, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(100);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1000);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(1, 90, 7, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            mecanumMove(0.5, -90, 12, 5);
            sleep(100);
            mecanumMove(0.5, 90, 8, 5);
            robot.FoundationClaw.setPower(0.8); //Clamp the foundation
            encoderDrives(0.6, 23, 23, 5); //Drive forward to clamp the foundation
            encoderDrives(1, -60, -50, 5); //Drive back with the foundation
            robot.FoundationClaw.setPower(-1);
            sleep(500);
            mecanumMove(1, 100, 50, 5);
            robot.FoundationClaw.setPower(0);
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}