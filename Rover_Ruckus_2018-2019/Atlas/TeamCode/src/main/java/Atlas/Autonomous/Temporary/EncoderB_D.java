package Atlas.Autonomous.Temporary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "EnocderB_D", group = "Auto")
public class EncoderB_D extends AggregatedClass {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        movement();
    }


    public void movement() {
        //Landing
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        sleep(2000);
        encoderDrives(0.5, 1, -1);
        sleep(250);
        robot.Latching.setPower(-1);
        sleep(500);
        encoderDrives(0.5, -1, 1);
        sleep(250);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();

        //Enocders
        encoderDrives(0.5, 31, 31);
        encoderDrives(0.5, -12, -12);
        encoderDrives(0.5, -12, 12);
        encoderDrives(0.5, 46, 46);
        encoderDrives(0.5, -5, 5);
        encoderDrives(0.5, 20, 20);

        //ColorSensor
        markerBD();
    }
}
