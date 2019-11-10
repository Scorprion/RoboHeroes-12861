package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.AggregatedHermes;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "HermesC", group = "Autonomous")
public class HermesC extends AggregatedHermes {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        encoderDrives(0.25, 27, 27,2);
        encoderDrives(0.25, -12, 12,2);
        encoderDrives(0.25, 27, 27,2);
    }
}