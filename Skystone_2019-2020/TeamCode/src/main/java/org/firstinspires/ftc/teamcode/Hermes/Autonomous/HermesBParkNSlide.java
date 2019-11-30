package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesBParkNSlide", group = "Hermes")

public class HermesBParkNSlide extends HermesAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        MecTime(-0.5,-0.5,-0.5,-0.5,900);
        stopMotors();
        sleep(20000);
        MecTime(0.5,0.5,0.5,0.5,2000);
        stopMotors();
        sleep(1000);
        mecanumMove(0.5,270, 36, 5);



    }
}