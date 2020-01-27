package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesAs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA_Place", group = "Hermes")
public class HermesA_Place extends HermesAggregated {
    public boolean VuforiaFound = false;
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        isD = true;
        //init_vuforia();

        waitForStart();

        encoderDrives(0.4, 27,27,5);
        sleep(100);
        mecanumMove(-0.4, 90, 31, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor();
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
            mecanumMove(0.4, 90, 6, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -95, -95, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);

            //Second SkyStone
            encoderDrives(0.65, 71, 71, 10); //Drive back to the second SkyStone
            mecanumMove(0.6, 90, 3, 5); //Strafe away from SkyStone for space for Clamp
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25); //Recenter with PID
            sleep(250);
            encoderDrives(0.65, -72, -72, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);

            //Parking
            encoderDrives(1, 29, 29, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
            telemetry.addLine("FIRST");

        } else if(pos == position.MIDDLE) { //*******************************************************************************
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, -8, -8, 5); //Move to the 2nd Position
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 6, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -87, -87, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, 63, 63, 10); //Drive back to the second SkyStone
            mecanumMove(0.6, 90, 3, 5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, -64, -64, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);

            //Parking
            encoderDrives(1, 29, 29, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
            telemetry.addLine("SECOND");

        } else if(pos == position.BRIDGE) { //*******************************************************************************
            //First SkyStone
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, -16, -16, 5); //Move to the 3rd Position
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 6, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -79, -79, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);

            //Second SkyStone
            encoderDrives(0.65, 55, 55, 10); //Drive back to the second SkyStone
            mecanumMove(0.6, 90, 3, 5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 10, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, -5, 5); //Strafe away from the SkyStone
            pidTurn(P, I, D, 0, 0, 1.25);
            sleep(250);
            encoderDrives(0.65, -56, -56, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            mecanumMove(0.4, 90, -6, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);

            //Parking
            encoderDrives(1, 29, 29, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}