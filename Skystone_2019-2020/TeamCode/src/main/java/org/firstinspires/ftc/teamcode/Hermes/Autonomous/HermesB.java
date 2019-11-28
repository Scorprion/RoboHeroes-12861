package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesB", group = "Hermes")
@Disabled
public class HermesB extends HermesAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        encoderDrives(0.5, 53, 53, 5);
        pidTurn(1.5, 0.2, 0, 90, 0.4, 2);
        encoderDrives(0.4, 10, 10, 2);
        robot.FoundationClaw.setPower(1);
        sleep(1000);
        robot.FoundationClaw.setPower(-1);
        pidTurn(1.5, 0.2, 0, 180, 0.4, 5);
        mecanumMove(0.5, 90, 9, 3);



    }
}