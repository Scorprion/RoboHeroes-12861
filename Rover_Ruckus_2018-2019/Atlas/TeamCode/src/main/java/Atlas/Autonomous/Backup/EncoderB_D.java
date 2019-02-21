package Atlas.Autonomous.Backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.Aggregated;
import Atlas.Autonomous.Init.Backup_Agg;
import Atlas.Autonomous.Init.Backup_Agg;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "EnocderB_D", group = "Auto")
public class EncoderB_D extends Aggregated {

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
        encoderDrives(0.4, 19.75, 19.75);
        sleep(100);
        BD_CS();
        if (!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(100);
            encoderDrives(0.3, -7, 7);
            sleep(100);
            encoderDrives(0.4, 8, 8);
            sleep(100);
            encoderDrives(0.2, 3, -3);
            sleep(100);
            encoderDrives(0.4, 4, 4);
            sleep(100);
            BD_CS();
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
