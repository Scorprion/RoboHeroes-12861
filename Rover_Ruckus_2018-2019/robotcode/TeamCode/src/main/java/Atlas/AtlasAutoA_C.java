package Atlas;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

@Autonomous(name = "AtlasAutoA_C", group = "Atlas")
public class AtlasAutoA_C extends LinearOpMode {

    HardwareAtlas robot = new HardwareAtlas();
    public boolean colorFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        movement();

    }

    public void cs(){
        //Making sure the motors go forward at 0.3 power
        robot.Left.setPower(0.2);
        robot.Right.setPower(0.2);
        //added second pause also for debugging
        sleep(250);
        while(!colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            NormalizedRGBA colors = robot.ColorSensor2.getNormalizedColors();
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
            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold
            if(Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(700);
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                sleep(1000);
                robot.Marker.setPosition(0.5);
                sleep(1000);
                colorFound = true;
            }
        }
    }

    public  void movement() {
        forward(0.4, 1.6); //Move forward 0.4 power for 3 seconds
        cs(); //Uses the color sensor method to stop and drop the marker
        turn(-0.4, 1.15);
        forward(0.4, 4);
        stopMotion(); //stop all motion
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
