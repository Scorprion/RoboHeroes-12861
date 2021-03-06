package Atlas.Autonomous.Backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "EncoderA_C", group = "Auto")
public class EncoderA_C extends Aggregated {

    //Use the AtlasEncoderDrive class to control the encoders

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Turn on light through the program
        if (robot.ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.ColorSensor).enableLight(true);
        }

        waitForStart();
        robot.LClamp.setPosition(0);
        sleep(200);
        movement();
    }


    public void movement() throws InterruptedException {
        //Landing
        robot.LClamp.setPosition(0);
        robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        robot.Sliding.setPosition(1);
        sleep(2000);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);

        //Encoders
        robot.Left.setPower(-0.15);
        robot.Right.setPower(-0.15);
        sleep(100);
        robot.Left.setPower(0.15);
        robot.Right.setPower(0.15);
        sleep(1000);
        encoderDrives(0.4, 19.5, 19.5);
        sleep(100);
        AC_CS();

        if(!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(100);
            encoderDrives(0.3, -7, 7);
            sleep(100);
            encoderDrives(0.4, 8, 8);
            sleep(100);
            encoderDrives(0.2, 3.3, -3.3);
            sleep(100);
            encoderDrives(0.4, 4.3, 4.3);
            sleep(100);
            AC_CS();

            if(!colorFound) {
                sleep(100);
                encoderDrives(0.4, -11, -11);
                sleep(100);
                encoderDrives(0.3, 11, -11);
                sleep(100);
                encoderDrives(0.4, 12, 12);
                rightAC();
            }
        }
    }
}
