package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Autonomous(name = "ColorTesting", group = "RoboBot")
public class ColorTesting extends LinearOpMode {

    HardwareMapInit robot  = new HardwareMapInit();   // Use a Pushbot's hardware
    public float previousRed = 1000, previousBlue = 1000;
    public NormalizedColorSensor Color_Sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Turn on light through the program
        if (Color_Sensor instanceof SwitchableLight) {
            ((SwitchableLight) Color_Sensor).enableLight(true);
        }
        //Color_Sensor2 = hwmap.get(NormalizedColorSensor.class, "ColorSensor2");
        waitForStart();
        cs();
    }

    public void cs() {
        while(opModeIsActive() || true) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = Color_Sensor.getNormalizedColors();
            NormalizedRGBA colors2 = robot.Color2.getNormalizedColors();
            NormalizedRGBA colors3 = robot.Color3.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            int color2 = colors2.toColor();
            int color3 = colors3.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a1", "%02x", Color.alpha(color))
                    .addData("r1", "%02x", Color.red(color))
                    .addData("g1", "%02x", Color.green(color))
                    .addData("b1", "%02x", Color.blue(color))
                    .addData("a2", "%02x", Color.alpha(color2))
                    .addData("r2", "%02x", Color.red(color2))
                    .addData("g2", "%02x", Color.green(color2))
                    .addData("b2", "%02x", Color.blue(color2))
                    .addData("a3", "%02x", Color.alpha(color3))
                    .addData("r3", "%02x", Color.red(color3))
                    .addData("g3", "%02x", Color.green(color3))
                    .addData("b3", "%02x", Color.blue(color3));
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            telemetry.addLine("normalized color:  ")
                    .addData("a1", Color.alpha(color))
                    .addData("r1", Color.red(color))
                    .addData("g1", Color.green(color))
                    .addData("b1", Color.blue(color));
            telemetry.addLine("");
                    telemetry.addData("a2", Color.alpha(color2))
                    .addData("r2", Color.red(color2))
                    .addData("g2", Color.green(color2))
                    .addData("b2", Color.blue(color2));
            telemetry.addLine("");
                    telemetry.addData("a3", Color.alpha(color3))
                    .addData("r3", Color.red(color3))
                    .addData("g3", Color.green(color3))
                    .addData("b3", Color.blue(color3));
            telemetry.update();
            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold
            if(Color.blue(color) >= 125 || Color.red(color) >= 140) {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
            }
            sleep(100);
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
