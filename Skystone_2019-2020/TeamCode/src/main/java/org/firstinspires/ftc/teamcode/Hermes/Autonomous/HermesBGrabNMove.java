package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesBGrabNMove", group = "Hermes")

public class HermesBGrabNMove extends HermesAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        mecanumMove(-0.5,90,24,5);
        stopMotors();
        sleep(1000);
        encoderDrives(-0.5,36,36,5);
        stopMotors();
        sleep(1000);
        robot.FoundationClaw.setPower(1);
        stopMotors();
        sleep(1000);
        encoderDrives(0.5,12,36,10);
        stopMotors();
        sleep(1000);
        encoderDrives(0.5,36,36,10);


    }
}