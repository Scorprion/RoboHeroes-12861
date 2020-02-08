package org.firstinspires.ftc.teamcode.Hermes.Tests;

import android.app.Activity;
import android.graphics.Color;
import android.provider.ContactsContract;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

import static java.lang.Math.abs;

@Config
@Autonomous(name= "ColorTester", group= "Pushbot")
public class ColorTester extends HermesAggregated {
    private static double color_cut = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();

        NormalizedColorSensor SkySensor1;
        NormalizedColorSensor SkySensor2;

        SkySensor1 = hardwareMap.get(NormalizedColorSensor.class, "SkySensor1");
        SkySensor2 = hardwareMap.get(NormalizedColorSensor.class, "SkySensor2");

        View relativeLayout;

        HardwareHermes robot = new HardwareHermes();


        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // Get a reference to our sensor object.

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (SkySensor1 instanceof SwitchableLight) {
            ((SwitchableLight) SkySensor1).enableLight(true);
        }

        while(opModeIsActive()) {
            NormalizedRGBA colors = SkySensor1.getNormalizedColors();
            NormalizedRGBA colors2 = SkySensor2.getNormalizedColors();

            double max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            int color1 = colors.toColor();

            double max2 = Math.max(Math.max(Math.max(colors2.red, colors2.green), colors2.blue), colors2.alpha);
            colors2.red   /= max2;
            colors2.green /= max2;
            colors2.blue  /= max2;
            int color2 = colors2.toColor();

            telemetry.addLine()
                    .addData("Color1 R:", Color.red(color1))
                    .addData("Color2 R:", Color.red(color2));
            telemetry.addLine()
                    .addData("Color1 G:", Color.green(color1))
                    .addData("Color2 G:", Color.green(color2));
            telemetry.addLine()
                    .addData("Color1 B:", Color.blue(color1))
                    .addData("Color2 B:", Color.blue(color2));
            telemetry.addLine()
                    .addData("Color1 A:", Color.alpha(color1))
                    .addData("Color2 A:", Color.alpha(color2));


            if(abs(Color.red(color1) - Color.red(color2)) < color_cut) {
                //Position 3
                telemetry.addLine("Position3");
            }else if (Color.red(color1) < Color.red(color2)){
                //Position 1
                telemetry.addLine("Position2");
            } else {
                //Position 2;
                telemetry.addLine("Position1");
            }
            telemetry.update();
        }
    }
}
