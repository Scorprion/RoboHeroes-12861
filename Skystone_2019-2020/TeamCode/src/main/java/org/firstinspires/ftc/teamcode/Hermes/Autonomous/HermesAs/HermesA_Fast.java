package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesAs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA_Fast", group = "Hermes")
public class HermesA_Fast extends HermesAggregated {
    private position pos = position.UNKNOWN;
    private double P = 2.0, I = 0.5, D = 0.08;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //init_vuforia();

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
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 114, 114, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.4, 90, 14, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 10, 3); //Strafe


            //Parking
            encoderDrives(1, -52, -52, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
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
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 106, 106, 10); //Move to the Build Zone to drop off the SkyStone
            mecanumMove(-0.4, 90, 14, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 10, 3);

            //Parking
            encoderDrives(1, -52, -52, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
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
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, 98, 98, 10); //Move to the Build Zone to drop off the SkyStone into Foundation
            mecanumMove(-0.4, 90, 14, 3); //Strafe closer to the foundation
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 10, 3);

            //Parking
            encoderDrives(1, -52, -52, 5); //Park
            mecanumMove(-0.4, 90, 4, 2); //Strafe closer to the bridge
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
    }