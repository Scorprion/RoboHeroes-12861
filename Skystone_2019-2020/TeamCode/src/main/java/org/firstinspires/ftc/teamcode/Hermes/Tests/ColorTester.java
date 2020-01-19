package org.firstinspires.ftc.teamcode.Hermes.Tests;

import android.app.Activity;
import android.graphics.Color;
import android.provider.ContactsContract;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;

@Autonomous(name= "ColorTester", group= "Pushbot")
public class ColorTester extends HermesAggregated {
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
            CheckSkySensor(false);
        }
    }
}
