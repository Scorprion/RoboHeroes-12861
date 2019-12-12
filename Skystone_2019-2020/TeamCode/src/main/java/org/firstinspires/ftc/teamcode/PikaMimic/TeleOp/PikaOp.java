package org.firstinspires.ftc.teamcode.PikaMimic.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PikaMimic.Autonomous.Init.Pikaware;

@TeleOp(name= "PikaOp", group= "Pushbot")
@Disabled
public class PikaOp extends OpMode {
    Pikaware pika = new Pikaware();
    public void init(){
        pika.init(hardwareMap);
    }

    public void loop(){
        double power = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        pika.Left.setPower(power + turn);
        pika.Right.setPower(power - turn);
    }

}
