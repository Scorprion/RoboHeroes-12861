package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PIDCoeffs;

import static org.firstinspires.ftc.teamcode.Hermes.HermesConstants.*;

@Config
@Autonomous(group="Hermes")
public class HeremesAuto extends HermesAggregated {
    public static double newX = 0, newY = 0, newAng = 90, time = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while(opModeIsActive()) {
            runTo(newX, newY, newAng, time, xPID, yPID, angPID);
            runTo(0, 0, 0, time, xPID, yPID, angPID);
        }
    }
}
