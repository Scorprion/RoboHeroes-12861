package org.firstinspires.ftc.teamcode.Skeleton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SkeletonControl", group = "Skeleton")
public class SkeletonControl extends SkeletonAggregated {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        mecanumMove(0.4, 90, 100, 2);
    }
}
