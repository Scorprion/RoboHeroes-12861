package Atlas.Autonomous.Temporary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "EncoderA_C", group = "Auto")
public class EncoderA_C extends AggregatedClass {

    //Use the AtlasEncoderDrive class to control the encoders

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        movement();
    }


    public void movement() throws InterruptedException {
        //Landing
        /*robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
        stopMotors();*/

        //Enocders
        encoderDrives(0.4, 19, 19);
        sleep(500);
        AC_CS();
        if(!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(500);
            encoderDrives(0.5, -0.5, 0.5);
            sleep(500);
            encoderDrives(0.4, 8, 8);
            sleep(500);
            encoderDrives(0.4, 2.25, -2.25);
            sleep(500);
            encoderDrives(0.4, 4.5, 4.5);
            sleep(500);
            AC_CS();
            if(!colorFound) {
                sleep(500);
                encoderDrives(0.4, -11, -11);
                sleep(500);
                encoderDrives(0.5, 12, -12);
                sleep(500);
                encoderDrives(0.4, 12, 12);
                leftAC();
            }
        }
    }
}
