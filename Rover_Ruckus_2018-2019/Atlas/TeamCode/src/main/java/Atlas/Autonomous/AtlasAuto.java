package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends Aggregated {

    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);
    private double output = 1; // Needs to be 100 to get passed the "while" statement's conditions
    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        /*PID(0.7, 0, 0, 90);
        while(opModeIsActive()) {
            angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Current Angle: ", angle);
            telemetry.update();
        }
        encoderDrives(0.4, 5, 5);

        PID(1.3, 0.01, 0.7, 88);

        encoderDrives(0.4, 10, 10);
        sleep(100000);*/

        while (opModeIsActive()) {
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.Right.getCurrentPosition(),
                    robot.Left.getCurrentPosition());
            telemetry.update();
        }
    }
}
