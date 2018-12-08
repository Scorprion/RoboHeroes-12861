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

import Atlas.HardwareAtlas;


@Autonomous(name = "AtlasAutoB_D", group = "Auto")
public class AtlasAutoB_D extends LinearOpMode {

    HardwareAtlas robot  = new HardwareAtlas();   // Use a Pushbot's hardware
    public boolean colorFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Turn on light through the program
        if (robot.ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.ColorSensor).enableLight(true);
        }

        waitForStart();
        movement();
    }

    public void cs() {
        //Making sure the motors go backward at 0.3 speed
        robot.Left.setPower(0);
        robot.Right.setPower(0);
        robot.Left.setPower(0.2); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(0.2);
        //added second pause also for debugging
        sleep(250);

        while(!colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            robot.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            telemetry.addLine("normalized color:  ")
                    .addData("a", Color.alpha(color))
                    .addData("r", Color.red(color))
                    .addData("g", Color.green(color))
                    .addData("b", Color.blue(color));
            telemetry.update();

            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold. After that, it drops our team marker
            if(Color.blue(color) >= 125 || Color.red(color) >= 140) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                sleep(1000);
                robot.Marker.setPosition(0); //Drops the team marker
                sleep(1000);
                //Set the boolean "colorFound" to true to stop the repeating while loop
                colorFound = true;
            }
        }
    }

    public void movement() {
        forward(0.4, 1.25); //Move forward 0.4 speed for 1.25 seconds
        sleep(1000);
        forward(-0.4, 0.2); //Move backward -0.4 speed for 0.2 seconds
        sleep(1000);
        robot.Left.setPower(0); //Debugging
        robot.Right.setPower(0);
        sleep(1000);
        turn(0.4, 0.5); //Turn ccw 0.4 speed for 0.5 seconds
        sleep(1000);
        forward(0.4, 1.75); //Move forward 0.4 speed for 1.75 seconds
        sleep(1000);
        turn(0.4, 0.45); //Turn ccw 0.4 speed for 0.45 seconds
        forward(0.3, 1); //Move forward 0.3 speed for 1 second
        cs(); //Uses the color sensor method to stop, drop, and turn the robot
        turn(0.3, 0.9); //Turn ccw 0.3 speed for 0.9 seconds
        robot.Left.setPower(-0.425); //Move backwards at a gradual slope for 3.5 seconds
        robot.Right.setPower(-0.5);
        sleep(3500);
        //stop all motion
        stopMotion();
    }

    public void forward(double speed, double seconds) {
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
