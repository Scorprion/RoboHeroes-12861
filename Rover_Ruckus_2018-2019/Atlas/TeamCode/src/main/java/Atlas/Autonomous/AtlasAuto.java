package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends Aggregated {

    private double output = 1; // Needs to be 100 to get passed the "while" statement's conditions
    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        PID(0.7, 0.1, 1, 90);
        while(opModeIsActive()) {
            angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Current Angle: ", angle);
            telemetry.update();
        }
        /*encoderDrives(0.4, 5, 5);

        PID(1.3, 0.01, 0.7, 88);

        encoderDrives(0.4, 10, 10);
        sleep(100000);*/
    }
}
