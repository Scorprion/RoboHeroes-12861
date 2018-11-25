package Atlas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

@Autonomous(name = "AtlasAutoA_C", group = "Atlas")
public class AtlasAutoB_D extends LinearOpMode {
    HardwareMapInit robot = new HardwareMapInit();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }
}
