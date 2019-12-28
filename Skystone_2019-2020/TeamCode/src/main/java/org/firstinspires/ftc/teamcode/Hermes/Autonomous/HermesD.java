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
        init_vuforia();

        waitForStart();

        //MecTime(-1, 1, 1, -1,500);
        //encoderDrives(0.3, 17, 17, 5);
        //sleep(500);

        mecanumMove(-0.3, 90, 100, 2.5);

        //VUFORIA CODE GOES HERE WHEN FIXED
        CheckSkySensor();

        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        encoderDrives(0.8, 55, 55, 8);
        sleep(500);
        robot.Gate.setPower(1);
        sleep(500);
        encoderDrives(0.4, 19, 19, 5);
        mecanumMove(0.4, 90, 12, 3);

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