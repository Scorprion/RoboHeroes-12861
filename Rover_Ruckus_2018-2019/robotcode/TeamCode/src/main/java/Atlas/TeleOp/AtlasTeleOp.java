package Atlas.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name= "AtlasTeleOp", group= "Pushbot")
public class AtlasTeleOp extends OpMode {

    public DcMotor Right;
    public DcMotor Left;
    public DcMotor RShoulder;
    public DcMotor LShoulder;
    public DcMotor LElbow;
    public DcMotor Winch1;
    public DcMotor Winch2;
    public Servo Clamp;
    public Servo Marker;
    double ShoulderSpeed = 0;
    double ElbowSpeed = 0;
    double turnspeed = 0;
    double speed = 0;

    //Initializing the motors for the arm


    //Set the speed of the motors when the left or right sticks are not idle

    public void init() {
        LShoulder = hardwareMap.dcMotor.get("LShoulder");
        LElbow = hardwareMap.dcMotor.get("LElbow");
        Marker = hardwareMap.servo.get("Marker");
        Clamp = hardwareMap.servo.get("Clamp");
        Right = hardwareMap.dcMotor.get("Right");
        Left = hardwareMap.dcMotor.get("Left");
        /*Winch1 = hardwareMap.dcMotor.get("Winch1");
        Winch2 = hardwareMap.dcMotor.get("Winch2");*/



        //DriveL.setDirection(DcMotor.Direction.REVERSE);
        LShoulder.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        turnspeed = gamepad1.left_stick_x * 1;
        speed = gamepad1.left_stick_y * 1;
        telemetry.addData("The speed for both motors", speed);
        telemetry.addData("The speed for both motors in turning", turnspeed);
        //Turning
        if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
            Left.setPower(turnspeed);
            Right.setPower(turnspeed);
        } else {
            Left.setPower(0);
            Right.setPower(0);
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            Left.setPower(-speed);
            Right.setPower(speed);
        } else {
            Left.setPower(0);
            Right.setPower(0);
        }

        //The shoulder
        if (gamepad2.left_stick_y != 0) {
            ShoulderSpeed = gamepad2.left_stick_y * 0.75;
            LShoulder.setPower(ShoulderSpeed);
        } else {
            ShoulderSpeed = 0;
        }

        //The elbow
        if (gamepad2.right_stick_y != 0) {
            ElbowSpeed = gamepad2.right_stick_y * -0.5;
            LElbow.setPower(ElbowSpeed);
        } else {
            ElbowSpeed = 0;
        }

        //The clamp
        if (gamepad2.x) {
            Clamp.setPosition(1);
            try {
                //Wait 250 milliseconds before stopping the movement of the clamp
                TimeUnit.MILLISECONDS.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Clamp.setPosition(0.5);
        }
        if (gamepad2.a) {
            Clamp.setPosition(0);
        }

        if(gamepad1.a) {
            Marker.setPosition(1.0);
        }
        if(gamepad1.b) {
            Marker.setPosition(0);
        }

        //The winch
        /*if (gamepad2.left_bumper) {
            Winch1.setPower(1);
            Winch2.setPower(1);
        } else {
            Winch1.setPower(0);
            Winch2.setPower(0);


        }*/

    }
}
