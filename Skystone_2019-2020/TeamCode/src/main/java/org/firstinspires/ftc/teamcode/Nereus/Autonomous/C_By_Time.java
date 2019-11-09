package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.Aggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "C_By_Time", group = "Autonomous")
public class C_By_Time extends Aggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);
        //Initial Movement
        TimeRun(1,1,2);
        sleep(500);
        TimeRun(-1,1,2.5);
        sleep(500);
        TimeRun(1,1,5);
        sleep(500);
        TimeRun(1,-1,2.5);
        sleep(500);
        TimeRun(1,1,2);


        //Capture
        robot.Arm.setPower(-1);
        sleep(500);
        TimeRun(-1,-1,2.5);
        sleep(500);
        TimeRun(0.5,-0.5,4);
        sleep(500);
        TimeRun(1,1,5);
        robot.Arm.setPower(0.5);

        //Parking
        TimeRun(-1,-1,5);
    }
}