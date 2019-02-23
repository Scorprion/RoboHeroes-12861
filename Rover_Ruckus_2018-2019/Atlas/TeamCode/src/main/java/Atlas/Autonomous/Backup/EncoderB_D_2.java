package Atlas.Autonomous.Backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "EncoderB_D_2", group = "Auto")
public class EncoderB_D_2 extends Aggregated {

    //Use the AtlasEncoderDrive class to control the encoders

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.LClamp.setPosition(0);
        sleep(250);
        movement();
    }


    public void movement() throws InterruptedException {
        //Landing
        robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        robot.Sliding.setPosition(1);
        sleep(3000);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);

        //Enocders
        robot.Left.setPower(-0.15);
        robot.Right.setPower(-0.15);
        sleep(500);
        robot.Left.setPower(0.15);
        robot.Right.setPower(0.15);
        sleep(1000);
        encoderDrives(0.4, 20, 20);
        sleep(500);
        BD_CS2();
        if (!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(500);
            encoderDrives(0.3, -7, 7);
            sleep(500);
            encoderDrives(0.4, 8, 8);
            sleep(500);
            encoderDrives(0.2, 3.25, -3.25);
            sleep(500);
            encoderDrives(0.4, 4, 4);
            sleep(500);
            BD_CS2();
            if (!colorFound) {
                sleep(500);
                encoderDrives(0.4, -11, -11);
                sleep(500);
                encoderDrives(0.3, 11, -11);
                sleep(500);
                encoderDrives(0.4, 12, 12);
                BD_CS2();
            }
        }
    }
}
