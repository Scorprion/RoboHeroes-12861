package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name = "HermesA", group = "Hermes")
public class HermesA extends HermesAggregated {
    private double speed = 0.1, pidOutput = 0;
    private VectorF locationV;
    public boolean VuforiaFound = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        init_vuforia();

        waitForStart();

        //MecTime(-1, 1, 1, -1,500);
        mecanumMove(-0.3, 90, 19, 3);
        sleep(500);
        start_vuforia();
        sleep(100);
        encoderDrives(0.2, -3, -3, 1);
        sleep(500);
        mecanumMove(-0.3, 90, 14, 4);
        sleep(500);
        robot.Gate.setPower(-0.5);
        sleep(500);
        mecanumMove(0.5, 90, 20, 5);
        sleep(500);
        robot.BackLeft.setPower(0.2);
        robot.FrontLeft.setPower(0.2);
        robot.BackRight.setPower(0.2);
        robot.FrontRight.setPower(0.2);
        LineReading();

    }
}