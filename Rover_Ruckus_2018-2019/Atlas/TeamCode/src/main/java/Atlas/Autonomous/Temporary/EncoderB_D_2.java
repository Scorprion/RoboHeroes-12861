package Atlas.Autonomous.Temporary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;

@Autonomous(name = "EnocderB_D_2", group = "Auto")
public class EncoderB_D_2 extends AggregatedClass {

    //Use the AtlasEncoderDrive class to control the encoders

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
        encoderDrives(0.5, 40, 40);


    }
}
