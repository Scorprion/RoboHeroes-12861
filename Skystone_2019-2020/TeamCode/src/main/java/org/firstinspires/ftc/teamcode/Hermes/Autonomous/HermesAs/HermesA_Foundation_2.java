package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesAs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA_Foundation_2", group = "Hermes")
public class HermesA_Foundation_2 extends HermesAggregated {
    public boolean VuforiaFound = false;
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        isD = true;
        //init_vuforia();

        waitForStart();

        encoderDrives(0.4, -27,-27,5);
        sleep(100);
        mecanumMove(-0.4, 90, 29, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor();
            telemetry.addLine("Checking position");
            telemetry.update();
        }

        if(pos == position.MIDDLE) {
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1400);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 7, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 112, 112, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.5, 90, 18, 5);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(0.4, 90, 6, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1.5); //PID turn to face the Foundation
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            sleep(150);
            encoderDrives(0.4, 9, 9, 3); //Drive forward to clamp the foundation
            robot.FoundationClaw.setPower(0);
            encoderDrives(0.5, -23, -23, 1.5); //Drive back with the foundation
            pidTurn(2.0, 0.5, 0.08, 0, 0, 3.0); //PID turn the foundation horizontally
            encoderDrives(0.4, 24, 20, 1.5); //Push the foundation towards the wall
            robot.FoundationClaw.setPower(-1);
            sleep(750);
            robot.FoundationClaw.setPower(0);
            mecanumMove(0.5, 90, 15, 3); //Strafe closer to the bridge
            encoderDrives(0.5, -47, -47, 5); //Park
            telemetry.addLine("FIRST");

        } else if(pos == position.WALL) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 8, 8, 5); //Move to the 2nd Position
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1400);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 6, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 103, 103, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.5, 90, 18, 5);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(0.4, 90, 6, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1.5); //PID turn to face the Foundation
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            sleep(150);
            encoderDrives(0.4, 9, 9, 3); //Drive forward to clamp the foundation
            robot.FoundationClaw.setPower(0);
            encoderDrives(0.5, -23, -23, 1.5); //Drive back with the foundation
            pidTurn(2.0, 0.5, 0.08, 0, 0, 3.0); //PID turn the foundation horizontally
            encoderDrives(0.4, 24, 20, 1.5); //Push the foundation towards the wall
            robot.FoundationClaw.setPower(-1);
            sleep(750);
            robot.FoundationClaw.setPower(0);
            mecanumMove(0.5, 90, 15, 3); //Strafe closer to the bridge
            encoderDrives(0.5, -47, -47, 5); //Park
            telemetry.addLine("SECOND");

        } else if(pos == position.BRIDGE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 16, 16, 5); //Move to the 3rd Position
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1400);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 6, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 95, 95, 10); //Move to the Build Zone to drop off the SkyStone into Foundation
            mecanumMove(-0.5, 90, 18,  5);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);

            //Foundation Repositioning
            mecanumMove(0.4, 90, 6, 2); //Strafe further away from Foundation
            pidTurn(P, I, D, 90, 0, 1.5); //PID turn to face the Foundation
            robot.FoundationClaw.setPower(1); //Clamp the foundation
            sleep(150);
            encoderDrives(0.4, 9, 9, 3); //Drive forward to clamp the foundation
            robot.FoundationClaw.setPower(0);
            encoderDrives(0.5, -23, -23, 1.5); //Drive back with the foundation
            pidTurn(2.0, 0.5, 0.08, 0, 0, 3.0); //PID turn the foundation horizontally
            encoderDrives(0.4, 24, 20, 1.5); //Push the foundation towards the wall
            robot.FoundationClaw.setPower(-1);
            sleep(750);
            robot.FoundationClaw.setPower(0);
            mecanumMove(0.5, 90, 15, 3); //Strafe closer to the bridge
            encoderDrives(0.5, -47, -47, 5); //Park
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}