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


@Autonomous(name = "AutonomousB_D_2", group = "Auto")
@Disabled
public class AutonomousB_D_2 extends LinearOpMode {

    HardwareMapInit robot  = new HardwareMapInit();   // Use a Pushbot's hardware

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



    public void movement() {
        forward(-0.4, 1.35);
        sleep(1000);
        turn(0.1,0.25);
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
