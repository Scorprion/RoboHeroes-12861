package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Created by 309413 on 8/24/2018.
 */

@TeleOp(name= "ExampleBot2019", group= "Pushbot")
public class TankDrivefor2019ExampleBot extends OpMode {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor Arm;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor Hacks; //Hacks is the relic Shovel
    //public ColorSensor colorsensor;
    public Servo RightServo;
    public Servo LeftServo;
    public DcMotor SensorArm;
    public Servo PushyObject;
    public Servo wtfudge;//wtfudge and wtfudge2 are the servos on the relic arm.
    public Servo wtfudge2;
    double speed = 0;
    double turn = 0;
    double Mechaturn = 0;
    double rise = 0;
    double ArmSpeed = 0;
    boolean toggle = false;


    public void init() {

        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        Arm = hardwareMap.dcMotor.get("flip");
        PushyObject = hardwareMap.servo.get("Door");

        backRight.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Say", "Press 'Start' and 'A' to begin.");
    }
    @Override
    public void loop() {

        Mechaturn = gamepad1.left_stick_x * 1;
        speed = gamepad1.left_stick_y * 1;
        rise = gamepad1.right_stick_y * 1;
        turn = gamepad1.right_stick_x * -1;
        ArmSpeed = gamepad2.right_stick_y;

        if (gamepad1.right_stick_x >= 0.1) {

            backLeft.setPower(-0.9 * turn);
            backRight.setPower(0.9 * turn);
        } else if (gamepad1.right_stick_x >= 0.9) {

            backLeft.setPower(1);
            backRight.setPower(-1);
        }
        if (gamepad1.right_stick_x <= -0.1) {

            backLeft.setPower(-0.9 * turn);
            backRight.setPower(0.9 * turn);
        } else if (gamepad1.right_stick_x <= -0.9) {

            backLeft.setPower(-1);
            backRight.setPower(1);
        }


        if(gamepad2.dpad_up) {
            PushyObject.setPosition(1);
        }if(gamepad2.dpad_down) {
            PushyObject.setPosition(0);
        }


        backLeft.setPower(speed);
        backRight.setPower(speed);
        Arm.setPower(-ArmSpeed);


    }
    public void init_loop() {


    }

    public void start() {

    }

    public void stop(){

        backLeft.setPower(0);
        backRight.setPower(0);

    }
}





