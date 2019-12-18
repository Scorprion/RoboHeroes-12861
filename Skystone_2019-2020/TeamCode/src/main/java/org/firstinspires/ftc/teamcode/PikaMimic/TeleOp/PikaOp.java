package org.firstinspires.ftc.teamcode.PikaMimic.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PikaMimic.Autonomous.Init.Pikaware;

@TeleOp(name= "PikaOp", group= "Pushbot")
public class PikaOp extends OpMode {
    DcMotor Left;
    DcMotor Right;

    public void init(){
        Right = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Left =  hardwareMap.get(DcMotor.class, "Left");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){


        double power = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        Left.setPower(power + turn);
        Right.setPower(power - turn);
    }

}
