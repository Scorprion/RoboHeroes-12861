package Atlas.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Atlas.Autonomous.Init.Hardware2019;

@TeleOp(name= "AtlasTeleOp", group= "Pushbot")
public class TeleOp2019 extends OpMode{



    public void init(){

    }

    public void loop(){
        Hardware2019 robot = new Hardware2019();
        robot.init(hardwareMap);

        int speed = 0;
        int turn = 0;
        gamepad1.right_stick_y = speed;
        gamepad1.left_stick_x = turn;

        robot.Left.setPower(speed - turn);
        robot.Right.setPower(speed + turn);

        if (gamepad1.a){
            robot.ClawL.setPower(1);

        }
        if (gamepad1.b){
            robot.ClawL.setPower(-1);

        }
        if (gamepad1.x){
            robot.ClawL.setPower(0);

        }
    }

}
