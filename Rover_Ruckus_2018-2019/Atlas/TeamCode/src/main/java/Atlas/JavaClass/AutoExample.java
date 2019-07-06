package Atlas.JavaClass;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoExample", group = "Atlas")
public class AutoExample extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("LeftMotor");
        right = hardwareMap.dcMotor.get("RightMotor");
        waitForStart();

        left.setPower(0.5);
    }
}

