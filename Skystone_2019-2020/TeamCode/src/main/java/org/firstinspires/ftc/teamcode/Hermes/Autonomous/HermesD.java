package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesD", group = "Hermes")
public class HermesD extends HermesAggregated {
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        isD = true;
        //init_vuforia();

        waitForStart();

        encoderDrives(0.4, 23,23,5);
        mecanumMove(-0.5, 90, 30,5);


        /*
        mecanumMove(-0.3, 90, 17, 2.5);
        pidTurn(2.5, 0.2, 0.1, 0, 0, 1.5);
        sleep(250);
        start_vuforia();
        sleep(500);
        encoderDrives(0.3, -2, -2, 1);
        sleep(250);
        robot.Gate.setPower(-0.4);
        sleep(250);
        mecanumMove(-0.3, 90, 12, 4);
        sleep(250);
        robot.Clamper.setPower(-1);
        sleep(1250);
        robot.Gate.setPower(0.5);
        sleep(250);
        mecanumMove(0.5, 90, 6, 5);
        sleep(250);
        encoderDrives(0.6, -80, -80, 10);
        sleep(250);
        mecanumMove(-0.3, 90, 13, 2);
        sleep(250);
        robot.Gate.setPower(-0.4);
        sleep(250);
        robot.Clamper.setPower(1);
        sleep(250);
        robot.Gate.setPower(0.4);
        mecanumMove(0.5, 90, 9, 1);
        sleep(250);
        encoderDrives(0.6, 54, 54, 5);

        /**
         * VUFORIA CODE PRESERVED
         */

        /*start_vuforia();

        sleep(100);
        encoderDrives(-0.2, 5, 5, 1);
        sleep(500);
        mecanumMove(-0.3, 90, 14, 4);
        sleep(500);
        robot.Gate.setPower(-1);*/
    }
}