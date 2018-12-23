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


@Autonomous(name = "AutonomousA_C_2", group = "Auto")
@Disabled
public class AutonomousA_C_2 extends LinearOpMode {

    HardwareMapInit robot  = new HardwareMapInit();   // Use a Pushbot's hardware
    public boolean colorFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Turn on light through the program
        if (robot.Color_Sensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.Color_Sensor).enableLight(true);
        }

        waitForStart();
        movement();
    }

    public void cs() {
        //Making sure the motors go backward at 0.3 speed
        robot.Left.setPower(0);
        robot.Right.setPower(0);

        robot.Left.setPower(-0.2);
        robot.Right.setPower(-0.2);

        //added second pause also for debugging
        sleep(250);
        while(!colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            robot.Color_Sensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = robot.Color_Sensor.getNormalizedColors();
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
                colorFound = true;
            }
        }
    }

    public void movement() {
        forward(-0.4, 1.25); //Move back -0.4 seconds for 1.25 seconds
        cs(); //Uses the color sensor method to stop, drop, and turn the robot

        forward(0.5, 1.4);
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
