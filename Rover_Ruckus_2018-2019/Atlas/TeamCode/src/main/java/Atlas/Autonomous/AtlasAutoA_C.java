package Atlas.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;


@Autonomous(name = "AtlasAutoA_C", group = "Auto")
public class AtlasAutoA_C extends AggregatedClass {

    public boolean colorFound = false;

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

    public void cs() {
        //Making sure the motors go forward at 0.2 speed
        sleep(1000);
        robot.Left.setPower(-0.2); //Move at 0.2 speed
        robot.Right.setPower(-0.2);
        sleep(250); //added second pause also for debugging
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            robot.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold. After that, it drops our team marker
            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold
            if (Color.green(color) >= 80 && Color.red(color) >= 90) {
                stopMotors();
                sleep(500);
                proportional(1, 4, 3);
                sleep(500);
                //Set the boolean "colorFound" to true to stop the repeating while loop
                colorFound = true;
            } else if (Color.red(color) >= 192 && Color.blue(color) >= 192 && Color.green(color) >= 192) {

            }
        }
    }

    public void cs2() {
        sleep(1000);
        robot.Left.setPower(0.8); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(0.8);
        sleep(250); //added second pause also for debugging
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            robot.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
            }


        }
    }

    public void movement() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.Latching.setPower(0.75);
        robot.Winch.setPower(-1);
        sleep(3250);
        robot.Latching.setPower(-1);
        sleep(500);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(2000);
        encoderDrives(1, 4, 4);
        proportional(1, 30, 4);
        sleep(1000);
        encoderDrives(1, 13, 13);
        sleep(5000);
        proportional(1, 90, 4);
        cs();
    }
}



   /* public void forward(double speed, double seconds) {
        double time = seconds * 1000;

        //Move at the speed "speed" and pause for "seconds" amount of time before stopping
        robot.Left.setPower(speed);
        robot.Right.setPower(speed);
        sleep((long)time);
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    public void turn(double speed, double seconds) {
        double time = seconds * 1000;

        //turn at speed "speed" and pause for "seconds" amount of time before stopping
        robot.Left.setPower(-speed);
        robot.Right.setPower(speed);
        sleep((long)time);
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    public void stopMotion() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }
}
*/
