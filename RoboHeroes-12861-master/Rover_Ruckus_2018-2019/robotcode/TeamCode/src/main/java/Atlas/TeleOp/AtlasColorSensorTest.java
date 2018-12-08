package Atlas.TeleOp;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import Atlas.HardwareAtlas;


@Autonomous(name = "AtlasColorSensorTest", group = "RoboBot")
public class AtlasColorSensorTest extends LinearOpMode {

    HardwareAtlas robot  = new HardwareAtlas();   // Use a Pushbot's hardware
    public float previousRed = 1000, previousBlue = 1000;
    public NormalizedColorSensor ColorSensor;
    public NormalizedColorSensor ColorSensor2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Turn on light through the program
        if (ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) ColorSensor).enableLight(true);
        }
        waitForStart();
        robot.Right.setPower(0.2);
        robot.Left.setPower(0.2);
        cs();
    }

    public void cs() {
        while(opModeIsActive()) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            ColorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor2");
            NormalizedRGBA colors = ColorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = ColorSensor2.getNormalizedColors();
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
            int color2 = colors2.toColor();

            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);

            telemetry.addLine("raw Android color2: ")
                    .addData("a", "%02x", Color.alpha(color2))
                    .addData("r", "%02x", Color.red(color2))
                    .addData("g", "%02x", Color.green(color2))
                    .addData("b", "%02x", Color.blue(color2));
            float max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);

            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            colors2.red /= max2;
            colors2.green /= max2;
            colors2.blue /= max2;
            color2 = colors2.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", Color.alpha(color))
                    .addData("r", Color.red(color))
                    .addData("g", Color.green(color))
                    .addData("b", Color.blue(color));
            telemetry.update();

            telemetry.addLine("normalized color2:  ")
                    .addData("a", Color.alpha(color2))
                    .addData("r", Color.red(color2))
                    .addData("g", Color.green(color2))
                    .addData("b", Color.blue(color2));
            telemetry.update();


            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold
            if(Color.blue(color) >= 125 || Color.red(color) >= 140) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
            }

            if(Color.blue(color2) >= 125 || Color.red(color2) >= 140) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
                sleep(1000);
                robot.Left.setPower(0.2);
                robot.Right.setPower(0.2);
                sleep(750);
            }
        }




        /*NormalizedRGBA colors2 = Color_Sensor2.getNormalizedColors();
        Color.colorToHSV(colors2.toColor(), hsvValues);
        int color2 = colors2.toColor();
        float max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);
        // Normalizing color sensor readings
        colors2.red /= max;
        colors2.green /= max;
        colors2.blue /= max;
        color = colors2.toColor();
        telemetry.addLine("Second normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        telemetry.update();
        // convert the RGB values to HSV values.
        Color.RGBToHSV(Color.red(color2), Color.green(color2), Color.blue(color2), hsvValues);*/
    }
}
