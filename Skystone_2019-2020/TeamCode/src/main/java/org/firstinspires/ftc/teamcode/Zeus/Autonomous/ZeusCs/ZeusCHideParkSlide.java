package org.firstinspires.ftc.teamcode.Zeus.Autonomous.ZeusCs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Zeus.Autonomous.Init.ZeusAggregated;

@Autonomous(name = "ZeusCHideParkSlide", group = "Hermes")
public class ZeusCHideParkSlide extends ZeusAggregated {

    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.StoneLift.setPosition(0.6);
        mecanumMove(0.2, 90, -4, 4);
        robot.StoneLift.setPosition(0.6);
        encoderDrives(0.6, 28, 28, 5);
        robot.StoneLift.setPosition(0.6);
        sleep(15000);
        encoderDrives(0.6, -66, -66 , 7);
        robot.StoneLift.setPosition(0.6);
        mecanumMove(0.3, 90, -25, 100);
    }
}