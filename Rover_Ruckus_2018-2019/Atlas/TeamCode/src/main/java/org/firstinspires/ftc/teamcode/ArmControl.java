package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "ArmControl", group= "Pushbot")
@Disabled
public class ArmControl extends OpMode {

    public DcMotor Shoulder;
    public DcMotor Elbow;
    public Servo Clamp;
    double ShoulderSpeed = 0;
    double ElbowSpeed = 0;

    public void init() {
        //Initializing the motors for the arm
        Shoulder = hardwareMap.dcMotor.get("ShoulderL");
        Elbow = hardwareMap.dcMotor.get("Elbow");
        Clamp = hardwareMap.servo.get("Clamp");
    }

    @Override
    public void loop() {

        //Set the speed of the motors when the left or right sticks are not idle
        if(gamepad1.left_stick_y != 0) {
            ShoulderSpeed = gamepad1.left_stick_y * 0.75;
            Shoulder.setPower(ShoulderSpeed);
        } else {
            ShoulderSpeed = 0;
        }

        if(gamepad1.right_stick_y != 0) {
            ElbowSpeed = gamepad1.right_stick_y * -0.5;
            Elbow.setPower(ElbowSpeed);
        } else {
            ElbowSpeed = 0;
        }

        if(gamepad1.x){
            Clamp.setPosition(1);
        }

        if(gamepad1.y) {
            Clamp.setPosition(0.5);
        }
        if(gamepad1.a) {
            Clamp.setPosition(0);
        }
        if(gamepad1.dpad_down) {
            Elbow.setPower(ElbowSpeed * -0.5);
        }

    }
}
