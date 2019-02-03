package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends AggregatedClass {

    private double output = 1; // Needs to be 100 to get passed the "while" statement's conditions

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        PID(1.3, 0.01, 0.7, 60);
        sleep(100000);
        /*encoderDrives(0.4, 5, 5);

        PID(1.3, 0.01, 0.7, 88);

        encoderDrives(0.4, 10, 10);
        sleep(100000);*/
    }
}
