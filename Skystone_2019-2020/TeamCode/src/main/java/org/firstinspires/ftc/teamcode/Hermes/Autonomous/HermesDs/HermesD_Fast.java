package org.firstinspires.ftc.teamcode.Hermes.Autonomous.HermesDs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesD_Fast", group = "Hermes")
public class HermesD_Fast extends HermesAggregated {
    public boolean VuforiaFound = false;
    private position pos = position.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        isD = true;

        waitForStart();

        encoderDrives(0.4, 27,27,5);
        sleep(100);
        mecanumMove(-0.4, 90, 31, 5);
        while(opModeIsActive() && pos == position.UNKNOWN) {
            pos = CheckSkySensor(false);
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
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -114, -114, 10); //Move to the Foundation
            mecanumMove(-0.7, 90, 10, 5);
            //mecanumMove(-1, 0, 115, 10);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.7, 90, 10, 5);
            encoderDrives(0.65, 46, 46, 10); //Park
            mecanumMove(-0.4, 90,4,5);
            telemetry.addLine("FIRST");
        } else if(pos == position.MIDDLE) { //*******************************************************************************
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.4, -8, -8, 5); //Move to the 3rd Position
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            mecanumMove(-0.4, 90, 4, 4); //Strafe closer to the SkyStone
            robot.Clamper.setPower(-1); //Clamp the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -106, -106, 10); //Move to the Foundation
            mecanumMove(-0.7, 90, 10, 5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.7, 90, 10, 5);
            encoderDrives(0.65, 46, 46, 10); //Park
            mecanumMove(-0.4, 90,4,5);
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
            mecanumMove(0.4, 90, 4, 4); //Strafe away from the SkyStone
            sleep(250);
            encoderDrives(0.65, -98, -98, 10); //Move to the Foundation
            mecanumMove(-0.7, 90, 10, 5);
            robot.Gate.setPower(-0.4); //Lower the Arm
            sleep(250);
            robot.Clamper.setPower(1); //Release the SkyStone
            sleep(1250);
            robot.Gate.setPower(0.5); //Lift up the Arm
            sleep(250);
            mecanumMove(0.7, 90, 10, 5);
            encoderDrives(0.65, 46, 46, 10); //Park
            mecanumMove(-0.4, 90,4,5);
            telemetry.addLine("THIRD");
        }
        telemetry.update();
        sleep(10000);
    }
}