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
    //public NormalizedColorSensor Color_Sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Turn on light through the program
        //if (Color_Sensor instanceof SwitchableLight) {
        //    ((SwitchableLight) Color_Sensor).enableLight(true);
        //}
        //Color_Sensor2 = hwmap.get(NormalizedColorSensor.class, "ColorSensor2");
        waitForStart();
        cs();
    }

    public void cs() {
        while(opModeIsActive() || true) {
            float[] hsvValues1 = new float[3];
            float[] hsvValues2 = new float[3];
            float[] hsvValues3 = new float[3];
            float[] hsvValues4 = new float[3];
            final float values1[] = hsvValues1;
            final float values2[] = hsvValues2;
            final float values3[] = hsvValues3;
            final float values4[] = hsvValues4;

            NormalizedRGBA ColorSensor = robot.Color_Sensor.getNormalizedColors();
            NormalizedRGBA colorsGround = robot.Color1.getNormalizedColors();
            NormalizedRGBA colorSample1 = robot.Color1.getNormalizedColors();
            NormalizedRGBA colorSample2 = robot.Color2.getNormalizedColors();

            Color.colorToHSV(ColorSensor.toColor(), hsvValues3);
            int color1 = ColorSensor.toColor();

            Color.colorToHSV(colorsGround.toColor(), hsvValues3);
            int color2 = colorsGround.toColor();

            Color.colorToHSV(colorSample1.toColor(), hsvValues3);
            int color3 = colorSample1.toColor();

            Color.colorToHSV(colorSample2.toColor(), hsvValues4);
            int color4 = colorSample2.toColor();

            telemetry.addLine("Ground Front raw Android color: ")
                    .addData("a2", "%02x", Color.alpha(color1))
                    .addData("r2", "%02x", Color.red(color1))
                    .addData("g2", "%02x", Color.green(color1))
                    .addData("b2", "%02x", Color.blue(color1));

            telemetry.addLine("Ground Back raw Android color: ")
                    .addData("a2", "%02x", Color.alpha(color2))
                    .addData("r2", "%02x", Color.red(color2))
                    .addData("g2", "%02x", Color.green(color2))
                    .addData("b2", "%02x", Color.blue(color2));

            telemetry.addLine("Sampler1 raw Android color: ")
                    .addData("a2", "%02x", Color.alpha(color3))
                    .addData("r2", "%02x", Color.red(color3))
                    .addData("g2", "%02x", Color.green(color3))
                    .addData("b2", "%02x", Color.blue(color3));

            telemetry.addLine("Sampler2 raw Android color: ")
                    .addData("a2", "%02x", Color.alpha(color4))
                    .addData("r2", "%02x", Color.red(color4))
                    .addData("g2", "%02x", Color.green(color4))
                    .addData("b2", "%02x", Color.blue(color4));

            float max1 = Math.max(Math.max(Math.max(ColorSensor.red, ColorSensor.green), ColorSensor.blue), ColorSensor.alpha);
            float max2 = Math.max(Math.max(Math.max(colorsGround.red, colorsGround.green), colorsGround.blue), colorsGround.alpha);
            float max3 = Math.max(Math.max(Math.max(colorSample1.red, colorSample1.green), colorSample1.blue), colorSample1.alpha);
            float max4 = Math.max(Math.max(Math.max(colorSample2.red, colorSample2.green), colorSample2.blue), colorSample2.alpha);

            ColorSensor.red /= max1;
            ColorSensor.green /= max1;
            ColorSensor.blue /= max1;

            colorsGround.red /= max2;
            colorsGround.green /= max2;
            colorsGround.blue /= max2;

            colorSample1.red /= max3;
            colorSample1.green /= max3;
            colorSample1.blue /= max3;

            colorSample2.red /= max4;
            colorSample2.green /= max4;
            colorSample2.blue /= max4;

            color1 = ColorSensor.toColor();
            color2 = colorsGround.toColor();
            color3 = colorSample1.toColor();
            color4 = colorSample2.toColor();


            telemetry.addLine("Ground Front normalized color:  ")
                    .addData("a1", Color.alpha(color1))
                    .addData("r1", Color.red(color1))
                    .addData("g1", Color.green(color1))
                    .addData("b1", Color.blue(color1));
            telemetry.addLine("Ground Backnormalized color:  ")
                    .addData("a2", Color.alpha(color2))
                    .addData("r2", Color.red(color2))
                    .addData("g2", Color.green(color2))
                    .addData("b2", Color.blue(color2));
            telemetry.addLine("Sampler1 normalized color:  ")
                    .addData("a3", Color.alpha(color3))
                    .addData("r3", Color.red(color3))
                    .addData("g3", Color.green(color3))
                    .addData("b3", Color.blue(color3));
            telemetry.addLine("Sampler2 normalized color:  ")
                    .addData("a4", Color.alpha(color4))
                    .addData("r4", Color.red(color4))
                    .addData("g4", Color.green(color4))
                    .addData("b4", Color.blue(color4));


            telemetry.update();
            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold
            sleep(100);
            //-------------This is for the sensor from the 3rd port
            if(((Color.red(color3)+ Color.green(color3))/2 )>= Color.blue(color3) && Color.red(color3) >= 90 && Color.green(color3) >= 80 ){
                telemetry.addData("Sampler1", "Yellow");
            }else if(Color.alpha(color3) >= 200){
                telemetry.addData("Sampler1", "White");
            }else if (Color.green(color3) >= Color.red(color3) && Color.green(color3) >= Color.blue(color3) ){
                telemetry.addData("Sampler1", "Green");
            }else if(Color.blue(color3) >= Color.red(color3) && Color.blue(color3) >= Color.green(color3)){
                telemetry.addData("Sampler1", "Blue");
            }else if(Color.red(color3) >= Color.green(color3) && Color.red(color3) >= Color.blue(color3)){
                telemetry.addData("Sampler1", "Red");
            }
            //-------------This is for the sensor from the 4th port
            if(((Color.red(color4)+ Color.green(color4))/2 )>= Color.blue(color4) && Color.red(color4) >= 90 && Color.green(color4) >= 80 ){
                telemetry.addData("Sampler2", "Yellow");
            }else if(Color.alpha(color4) >= 200){
                telemetry.addData("Sampler2", "White");
            }else if (Color.green(color4) >= Color.red(color4) && Color.green(color4) >= Color.blue(color4) ){
                telemetry.addData("Sampler2", "Green");
            }else if(Color.blue(color4) >= Color.red(color4) && Color.blue(color4) >= Color.green(color4)){
                telemetry.addData("Sampler2", "Blue");
            }else if(Color.red(color4) >= Color.green(color4) && Color.red(color4) >= Color.blue(color4)){
                telemetry.addData("Sampler2", "Red");
            }
        }

        
        
        
        /*NormalizedRGBA colorSample2 = Color_Sensor2.getNormalizedColors();
        Color.colorToHSV(colorSample2.toColor(), hsvValues);
        int color2 = colorSample2.toColor();
        float max2 = Math.max(Math.max(Math.max(colorSample2.red, colorSample2.green), colorSample2.blue), colorSample2.alpha);
        // Normalizing color sensor readings
        colorSample2.red /= max;
        colorSample2.green /= max;
        colorSample2.blue /= max;
        color = colorSample2.toColor();
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
