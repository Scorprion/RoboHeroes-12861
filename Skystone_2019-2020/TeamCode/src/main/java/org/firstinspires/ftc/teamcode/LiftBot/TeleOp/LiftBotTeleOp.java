package org.firstinspires.ftc.teamcode.LiftBot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "TeleOp LiftBot", group= "Pushbot")
public class LiftBotTeleOp extends OpMode {

    double upDown;

    CRServo Left;
    CRServo Right;

    public void init() {
        Left = hardwareMap.get(CRServo.class, "Left");
        Right = hardwareMap.get(CRServo.class, "Right");
    }

    public void loop(){
        upDown = gamepad1.left_stick_y;

        if (gamepad1.left_stick_y >= 0 || gamepad1.left_stick_y <= 0) {
            Left.setPower(upDown);
            Right.setPower(upDown);
        }else {
            Left.setPower(0);
            Right.setPower(0);
        }
    }
}
