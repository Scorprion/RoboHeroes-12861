package Atlas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends LinearOpMode {
    HardwareMapInit robot = new HardwareMapInit();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }
}
