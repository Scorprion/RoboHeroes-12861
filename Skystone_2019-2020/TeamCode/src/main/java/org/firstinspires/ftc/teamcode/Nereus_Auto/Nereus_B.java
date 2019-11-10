package org.firstinspires.ftc.teamcode.Nereus_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus_Auto.Init.Nereus_Aggregated;
import org.firstinspires.ftc.teamcode.Nereus_Auto.Init.PID;

@Autonomous(name = "Nereus_B", group = "Autonomous")
public class Nereus_B extends Nereus_Aggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);

        encoderDrives(0.25, 27, 27, 3000);
        encoderDrives(0.25, 12, -12, 3000);
        encoderDrives(0.25, 27, 27, 3000);
    }
}