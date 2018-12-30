package TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import TestBot.Init.HardwareTestBot;

@Autonomous(name = "GyroTest", group = "TestBot")
public class GyroTest extends AggregatedTestBot {
    HardwareTestBot robot = new HardwareTestBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        while(opModeIsActive()) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Angle:", robot.angles.firstAngle);
            telemetry.update();
        }
    }
}
