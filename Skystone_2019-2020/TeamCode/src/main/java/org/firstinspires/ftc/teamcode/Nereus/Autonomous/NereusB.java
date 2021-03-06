package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "NereusB", group = "Nereus")
@Disabled
public class NereusB extends NereusAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, null);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);

        encoderDrives(0.25, 27, 27, 3);
        encoderDrives(0.25, 9.425, -9.425, 2.5);
        encoderDrives(0.25, 14, 14, 3);
    }
}