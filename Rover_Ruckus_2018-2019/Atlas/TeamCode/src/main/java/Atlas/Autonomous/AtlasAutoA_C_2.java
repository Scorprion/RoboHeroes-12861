package Atlas.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

import static Atlas.Autonomous.Init.AggregatedClass.direction.CCW;
import static Atlas.Autonomous.Init.AggregatedClass.direction.CW;


@Autonomous(name = "AtlasAutoA_C_2", group = "Auto")
@Disabled
public class AtlasAutoA_C_2 extends AggregatedClass {

    public boolean colorFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
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
        AC_CS2();
        if(!colorFound) {
            encoderDrives(0.4, -6, -6);
            sleep(500);
            PIDCCW(0.4, 0.4, 0, 46);
            sleep(500);
            encoderDrives(0.4, 8, 8);
            sleep(500);
            encoderDrives(0.4, 2.25, -2.25);
            sleep(500);
            encoderDrives(0.4, 4.5, 4.5);
            sleep(500);
            AC_CS2();
            /*if(!colorFound) {
                sleep(500);
                encoderDrives(0.4, -11, -11);
                sleep(500);
                PIDCW(0.3, 0.4, 0, 320);
                sleep(500);
                encoderDrives(0.4, 12, 12);
                leftAC(2);
            } */
        }
    }


    public void stopMotion() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }
}
