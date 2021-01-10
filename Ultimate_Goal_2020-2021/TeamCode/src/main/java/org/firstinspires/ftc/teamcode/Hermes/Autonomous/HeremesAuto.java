package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PIDCoeffs;

@Config
@Autonomous(name="HermesAuto", group="Hermes")
public class HeremesAuto extends HermesAggregated {
    PIDCoeffs xPID = new PIDCoeffs(1, 0, 0);
    PIDCoeffs yPID = new PIDCoeffs(1, 0, 0);
    PIDCoeffs angPID = new PIDCoeffs(1, 0, 0);

    public static double newX = 3, newY = 3, newAng = 0, time = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        runTo(newX, newY, newAng, time, xPID, yPID, angPID);

    }
}
