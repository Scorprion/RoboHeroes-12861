package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "Hermes_A", group = "Autonomous")
public class NereusA extends NereusAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);

        encoderDrives(0.3, 22, 22, 2);

        /*pid.setParams(1.45, 0.5, 0, 270);
        run_pid(0, 5500, pid, true);

        encoderDrives(0.3, 18,18);

        pid.setParams(1.52, 0.5, 0, 0);
        run_pid(0, 5500, pid, true);*/

        encoderDrives(0.3, 10, 10,2);
        robot.Arm.setPower(-1);
        sleep(250);
        robot.Arm.setPower(0);
        encoderDrives(0.3, -10, -10,1);
        encoderDrives(0.2, -14, 14,1);
        encoderDrives(0.5, 103,103,4);
        robot.Arm.setPower(0.7);
        sleep(750);
        robot.Arm.setPower(0);
        encoderDrives(0.5, -60, -60,3.5);
    }
}