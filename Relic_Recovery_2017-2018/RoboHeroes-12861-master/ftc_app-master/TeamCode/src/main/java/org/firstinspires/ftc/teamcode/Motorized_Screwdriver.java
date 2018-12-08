package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.CameraDevice;

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
/**
 * Created by brick on 1/10/2018.
 */
@TeleOp(name = "Motorized Screwdriver", group = "Pushbot")
public class Motorized_Screwdriver extends OpMode {

    int var = 1;
    double speed1 = 0;
    double speed2 = 0;
    double speed3 = 0;
    public DcMotor ScrewDriver;
    public DcMotor Helper1;
    public DcMotor Helper2;
    public DcMotor Helper3;

    DigitalChannel Button1;
    //DigitalChannel Button2;

    @Override
    public void init() {
        ScrewDriver = hardwareMap.dcMotor.get("ScrewDriver");
        Helper1 = hardwareMap.dcMotor.get("Helper1");
        Helper2 = hardwareMap.dcMotor.get("Helper2");
        //Helper3 = hardwareMap.dcMotor.get("Helper3");
        telemetry.addData("Say", "Press 'Start' and 'A' to begin.");


    }

    @Override
    public void init_loop() {


    }
    @Override
    public void loop() {

        speed1 = gamepad1.right_stick_x;
        speed2 = gamepad1.left_stick_x;
        //speed3 = gamepad1.right_trigger;
        //speed3 = gamepad1.left_trigger * -1;

        Button1 = hardwareMap.get(DigitalChannel.class, "Button1");

        Button1.setMode(DigitalChannel.Mode.INPUT);

        if(gamepad1.a) {
            var = 1;
        }

        if(gamepad1.b) {
            var = -1;
        }

        if(Button1.getState() == false) {
            ScrewDriver.setPower(var);

            telemetry.addData("Hi","This is working");
        } else {
            ScrewDriver.setPower(0);
        }


            Helper1.setPower(speed1);
            Helper2.setPower(speed2);
            //Helper3.setPower(speed3);

            telemetry.update();

    }

}