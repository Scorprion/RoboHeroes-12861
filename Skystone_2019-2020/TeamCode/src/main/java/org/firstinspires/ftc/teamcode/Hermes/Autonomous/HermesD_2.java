package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesD_2", group = "Hermes")
public class HermesD_2 extends HermesAggregated {
    public boolean VuforiaFound = false;
    private position pos = position.UNKNOWN;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        isD = true;
        init_vuforia();

        waitForStart();

        encoderDrives(0.4, 27,27,5);
        mecanumMove(-0.4, 90, 31, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor(true);
            telemetry.addLine("Checking position");
            telemetry.update();
        }

        if(pos == position.WALL) {
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
            mecanumMove(0.4, 90, 17, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -88, -88, 10); //Move to the Build Zone to drop off the SkyStone
            //mecanumMove(-1, 0, 115, 10);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            encoderDrives(0.65, 62, 62, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 25, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, 22, 5); //Strafe away from the SkyStone
            encoderDrives(0.65, -55, -55, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);
            encoderDrives(1, 14, 14, 5); //Park
            mecanumMove(0.4, 90, 5, 2); //Strafe closer to the wall
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
            mecanumMove(0.4, 90, 17, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -80, -80, 10); //Move to the Build Zone to drop off the SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            encoderDrives(0.65, 54, 54, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 25, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, 22, 5); //Strafe away from the SkyStone
            encoderDrives(0.65, -47, -47, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);
            encoderDrives(1, 14, 14, 5); //Park
            mecanumMove(0.4, 90, 5, 2); //Strafe closer to the wall
            telemetry.addLine("SECOND");
        } else if(pos == position.BRIDGE) { //*******************************************************************************
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
            mecanumMove(0.4, 90, 17, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -72, -72, 10); //Move to the Build Zone to drop off the SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            encoderDrives(0.65, 46, 46, 10); //Drive back to the second SkyStone
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.6, 90, 25, 5); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.6, 90, 22, 5); //Strafe away from the SkyStone
            encoderDrives(0.65, -39, -39, 10); //Drive back to the Build Zone to drop off 2nd SkyStone
            robot.Gate.setPower(-0.4); // Drop off the SkyStone
            sleep(250);
            robot.Clamper.setPower(1);
            sleep(1250);
            robot.Gate.setPower(0.6);
            sleep(250);
            encoderDrives(1, 14, 14, 5); //Park
            mecanumMove(0.4, 90, 5, 2); //Strafe closer to the wall
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}