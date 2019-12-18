package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "NereusA", group = "Nereus")
@Disabled
public class NereusA extends NereusAggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0, null);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);

        encoderDrives(0.25, 14, 14, 2);
        encoderDrives(0.3, -9.425, 9.425, 2.5);
        vuforia(0.05, -200);
        encoderDrives(0.2, -7, -7, 1.5);

        /*pid.setParams(1.45, 0.5, 0, 270);
        run_pid(0, 5500, pid, true);

        encoderDrives(0.3, 18,18);

        pid.setParams(1.52, 0.5, 0, 0);
        run_pid(0, 5500, pid, true);*/

        encoderDrives(0.3, 9.425, -9.425, 2.5);
        encoderDrives(0.3, 23, 23,2);
        robot.Arm.setPower(-1);
        sleep(500);
        robot.Arm.setPower(0);
        encoderDrives(0.4, -12, -12,1.5);
        encoderDrives(0.2, -11, 11,2.5);
        encoderDrives(0.7, 160, 160,5);
        encoderDrives(0.4, -5, -5, 1);
        robot.Arm.setPower(0.7);
        sleep(750);
        robot.Arm.setPower(0);
        encoderDrives(0.5, -55, -55,3.5);
    }
}