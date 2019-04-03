package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.Aggregated;

import static Atlas.Autonomous.Init.Aggregated.direction.CW;
import static Atlas.Autonomous.Init.Aggregated.direction.CCW;


@Autonomous(name = "AtlasAutoA_C_Test", group = "Test")
public class AtlasAutoA_C_Test extends Aggregated {
    boolean loop = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Turn on light through the program
        if (robot.ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.ColorSensor).enableLight(true);
        }


        waitForStart();
        robot.LClamp.setPosition(0);
        movement();
    }

    public void movement() throws InterruptedException {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        /*robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        sleep(2000);
        robot.Sliding.setPosition(1);
        sleep(500);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);*/
        //calibrateCS();
        encoderDrives(0.4, 19, 19);
        sleep(500);
        AC_CS();
        if(!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(500);
            //                      PID(0.4, 0.4, 0, 46);
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
                //                       PID(0.3, 0.4, 0, 320);
                sleep(500);
                encoderDrives(0.4, 12, 12);
                AC_CS();
            }
        }
    }
}

