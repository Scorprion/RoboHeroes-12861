package Atlas.Autonomous.Temporary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;

@Autonomous(name = "EncoderB_D_2", group = "Auto")
public class EncoderB_D_2 extends AggregatedClass {

    //Use the AtlasEncoderDrive class to control the encoders

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        movement();
    }


    public void movement() throws InterruptedException {
        //Landing
        /*robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        sleep(2000);
        robot.Sliding.setPosition(1);
        sleep(500);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);*/

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
            encoderDrives(0.4, 4.25, 4.25);
            sleep(500);
            BD_CS2();
            if (!colorFound) {
                sleep(500);
                encoderDrives(0.4, -11, -11);
                sleep(500);
                encoderDrives(0.3, 11, -11);
                sleep(500);
                encoderDrives(0.4, 12, 12);
                rightBD2();
            }
        }
    }
}
