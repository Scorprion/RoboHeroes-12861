package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesAs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Hermes")
public class HermesA extends HermesAggregated {
    public boolean VuforiaFound = false;
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        encoderDrives(0.4, -27, -27, 5);
        sleep(100);
        mecanumMove(-0.4, 90, 31, 5);
        while (opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor();
            telemetry.addLine("Checking position");
            telemetry.update();
        }

        if (pos == position.WALL) { //MIDDDLEEEEE
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 8, 8, 5); //Move to the 2nd Position
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 5, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 7, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 80, 80, 10); //Move to the Build Zone to drop off the SkyStone
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, -55, -55, 10); //Drive back to the second SkyStone
            mecanumMove(0.4, 90, 7, 1);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -7, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, 47, 47, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(-0.6);
            sleep(250);
            encoderDrives(1, -14, -14, 5); //Park
            mecanumMove(-0.4, 90, 10, 2); //Strafe closer to the bridge
            telemetry.addLine("SECOND");
        } else if (pos == position.MIDDLE) { //WAAAAAAAAAAALLLLLLLLL*******************************************************************************
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 5, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 7, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 88, 88, 10); //Move to the Build Zone to drop off the SkyStone
            //mecanumMove(-1, 0, 115, 10);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, -63, -63, 10); //Drive back to the second SkyStone
            mecanumMove(0.4, 90, 4, 1);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -7, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, 55, 55, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(-0.6);
            sleep(250);
            encoderDrives(1, -14, -14, 5); //Park
            mecanumMove(-0.4, 90, 10, 2); //Strafe closer to the bridge
            telemetry.addLine("FIRST");
        } else if (pos == position.BRIDGE) { //*******************************************************************************
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, 16, 16, 5); //Move to the 3rd Position
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 5, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 7, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 72, 72, 10); //Move to the Build Zone to drop off the SkyStone
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, -47, -47, 10); //Drive back to the second SkyStone
            mecanumMove(0.4, 90, 4, 1);
            robot.Gate.setPower(0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(-0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -7, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, 39, 39, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(-0.6);
            sleep(250);
            encoderDrives(1, -14, -14, 5); //Park
            mecanumMove(-0.4, 90, 10, 2); //Strafe closer to the bridge
            telemetry.addLine("THIRD");
        }
        telemetry.update();

        sleep(10000);
    }
}