package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Init.Aggregated;
import org.firstinspires.ftc.teamcode.Autonomous.Init.PID;

@Autonomous(name = "Auto_A", group = "Autonomous")
public class Auto_A extends Aggregated {

    private PID pid = new PID(0.5, 0.5, 0, 90);

    @Override
    public void runOpMode() throws InterruptedException {

        while(opModeIsActive()) {
            encoderDrives(0.2, 10, 10);
        }
    }
}
