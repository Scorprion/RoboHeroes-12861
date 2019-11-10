package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "NereusC", group = "Autonomous")
public class NereusC extends NereusAggregated {

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