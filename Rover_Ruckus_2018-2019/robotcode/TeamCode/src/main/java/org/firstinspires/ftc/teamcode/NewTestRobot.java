package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "NewTestRobot", group= "Pushbot")
@Disabled
public class NewTestRobot extends OpMode {

    public DcMotor DriveR;
    public DcMotor DriveL;
    double Speed = 0;
    double Turn = 0;



    public void init() {
        //Initializing the motors for the arm
        DriveR = hardwareMap.dcMotor.get("DriveR");
        DriveL = hardwareMap.dcMotor.get("DriveL");

        DriveR.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //Set the speed of the motors when the left or right sticks are not idle
        if(gamepad1.left_stick_y != 0) {
            Speed = gamepad1.left_stick_y * 1;
            DriveR.setPower(Speed);
            DriveL.setPower(Speed);
        } else if (gamepad1.left_stick_x == 0 || gamepad1.left_stick_y == 0){
            Speed = 0;
            DriveR.setPower(Speed);
            DriveL.setPower(Speed);
        }

        if (gamepad1.left_stick_x !=0)  {
            Turn = gamepad1.left_stick_x;
            DriveR.setPower(Turn);
            DriveL.setPower(-Turn);
        } else if (gamepad1.left_stick_y == 0 || gamepad1.left_stick_x == 0){
          Turn = 0;
          DriveR.setPower(Turn);
          DriveL.setPower(-Turn);
        }

    }
}
