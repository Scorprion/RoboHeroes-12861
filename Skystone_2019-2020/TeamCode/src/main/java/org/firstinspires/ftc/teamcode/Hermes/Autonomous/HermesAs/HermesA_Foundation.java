package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesAs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA_Foundation", group = "Hermes")
public class HermesA_Foundation extends HermesAggregated {
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        encoderDrives(0.4, -27,-27,5);
        sleep(100);
        mecanumMove(-0.4, 90, 31, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor(false);
            telemetry.addLine("Checking position");
            telemetry.update();
        }

        if(pos == position.WALL) {
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 3, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.8, 108, 108, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.4, 90, 12, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 6, 3);

            //Foundation Repositioning
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            encoderDrives(0.4, 11, 11, 3); //Drive forward to clamp the foundation
            encoderDrives(0.5, -26, -26, 3); //Drive back with the foundation
            robot.FoundationClaw.setPower(0);
            pidTurn(2.0, 0.5, 0.08, 0, 0, 2); //PID turn the foundation horizontally
            robot.FoundationClaw.setPower(-1); //UnClamp the foundation
            encoderDrives(0.7, 24, 20, 1.5); //Push the foundation towards the wall
            sleep(500);
            robot.FoundationClaw.setPower(0);
            sleep(200);
            mecanumMove(0.5, 90, 26, 4);
            encoderDrives(0.7, -47, -47, 5); //Park
            telemetry.addLine("FIRST");

        } else if(pos == position.MIDDLE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 8, 8, 5); //Move to the 2nd Position
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 3, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.8, 100, 100, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.4, 90, 8, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 5, 3);

            //Foundation Repositioning
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            encoderDrives(0.4, 11, 11, 3); //Drive forward to clamp the foundation
            encoderDrives(0.5, -26, -26, 3); //Drive back with the foundation
            robot.FoundationClaw.setPower(0);
            pidTurn(2.0, 0.5, 0.08, 0, 0, 2); //PID turn the foundation horizontally
            robot.FoundationClaw.setPower(-1); //UnClamp the foundation
            encoderDrives(0.7, 24, 20, 1.5); //Push the foundation towards the wall
            sleep(500);
            robot.FoundationClaw.setPower(0);
            sleep(200);
            mecanumMove(0.5, 90, 26, 4);
            encoderDrives(0.7, -47, -47, 5); //Park
            telemetry.addLine("SECOND");

        } else if(pos == position.BRIDGE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 16, 16, 5); //Move to the 3rd Position
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 3, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.8, 92, 92, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.4, 90, 8, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 5, 3);

            ///Foundation Repositioning
            pidTurn(P, I, D, 90, 0, 1); //PID turn to face the Foundation
            sleep(250);
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            encoderDrives(0.4, 11, 11, 3); //Drive forward to clamp the foundation
            encoderDrives(0.5, -26, -26, 3); //Drive back with the foundation
            robot.FoundationClaw.setPower(0);
            pidTurn(2.0, 0.5, 0.08, 0, 0, 2); //PID turn the foundation horizontally
            robot.FoundationClaw.setPower(-1); //UnClamp the foundation
            encoderDrives(0.7, 24, 20, 1.5); //Push the foundation towards the wall
            sleep(500);
            robot.FoundationClaw.setPower(0);
            sleep(200);
            mecanumMove(0.5, 90, 26, 4);
            encoderDrives(0.7, -47, -47, 5); //Park
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}