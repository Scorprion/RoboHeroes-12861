package org.firstinspires.ftc.teamcode.PikaMimic.TeleOp;

import org.firstinspires.ftc.teamcode.PikaMimic.Autonomous.Init.Pikaware;

@TeleOp(name= "PikaOp", group= "Pushbot")
public class PikaOp extends OpMode{

    public void init(){

    }

    public void loop(){
        double power = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        Pikaware.Left.setPower(power + turn);
        Pikaware.Right.setPower(power - turn);
    }

}
