package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.Aggregated;
import Atlas.Autonomous.Init.HardwareAtlas;

import static Atlas.Autonomous.Init.Aggregated.direction.CW;
import static Atlas.Autonomous.Init.Aggregated.direction.CCW;


@Autonomous(name = "AtlasAutoA_C", group = "Auto")
@Disabled
public class AtlasAutoA_C extends Aggregated {
    boolean loop = true;
    HardwareAtlas robot = new HardwareAtlas();

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

    public void movement() {
        try{
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }catch(Exception e){
            vuforia.init
        }
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
        encoderDrives(0.3, 6, 6);
        sleep(1000);
        // PID(0.5, 0.5, 0, 60);
        //proportional(CW, 0.5, 52, 3);
        sleep(500);
        encoderDrives(0.4, 26, 26);
        sleep(250);
        // PID(0.42, 0.5, 0, 88);
        sleep(500);
    }
}

