package TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import TestBot.Init.AggregatedTestBot;
import TestBot.Init.HardwareTestBot;

@Autonomous(name = "GyroTest", group = "TestBot")
public class GyroTest extends AggregatedTestBot {
    HardwareTestBot robot = new HardwareTestBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }
}
