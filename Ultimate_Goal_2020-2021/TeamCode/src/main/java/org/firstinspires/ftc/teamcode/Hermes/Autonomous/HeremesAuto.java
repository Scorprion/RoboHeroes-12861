package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name="HermesAuto", group="Hermes")
public class HeremesAuto extends HermesAggregated {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        runTo(3, 3, 0, new double[] {0.5, 0, 0}, new double[] {0.5, 0, 0}, new double[] {0.5, 0, 0});
    }
}
