package Atlas.Autonomous.DelayedAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "DelayedEnocderB_D", group = "Auto")
public class DelayedAtlasAutoB_D extends Aggregated {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.LClamp.setPosition(250);
        sleep(250);
        movement();
    }


    public void movement() throws InterruptedException{
        //Landing
        robot.LClamp.setPosition(0);
        robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        robot.Sliding.setPosition(1);
        sleep(3000);
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
        encoderDrives(0.4, 20.5, 19.5);
        sleep(100);
        DelayedBD_CS();

        if (!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(100);
            encoderDrives(0.3, -8, 8);
            sleep(100);
            encoderDrives(0.4, 8, 8);
            sleep(100);
            encoderDrives(0.2, 3, -3);
            sleep(100);
            encoderDrives(0.4, 3.5, 3.5);
            sleep(100);
            DelayedBD_CS();

            if (!colorFound) {
                sleep(100);
                encoderDrives(0.4, -11, -11);
                sleep(100);
                encoderDrives(0.3, 11, -11);
                sleep(100);
                encoderDrives(0.4, 12, 12);
                rightBD();
            }
        }
    }
}
