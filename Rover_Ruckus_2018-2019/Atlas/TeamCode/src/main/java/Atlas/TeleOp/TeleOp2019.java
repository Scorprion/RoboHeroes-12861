package Atlas.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Atlas.Autonomous.Init.Hardware2019;

@TeleOp(name= "TeleOp2019", group= "Pushbot")
public class TeleOp2019 extends OpMode {

    Hardware2019 robot = new Hardware2019();
    float speed = 0;
    float turn = 0;

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        if (gamepad1.right_stick_x >= 0 || gamepad1.right_stick_x <= 0 || gamepad1.right_stick_y >= 0 || gamepad1.right_stick_y <= 0) {
            speed = gamepad1.right_stick_y;
            turn = gamepad1.right_stick_x;
        }
        if (gamepad1.a) {
            robot.ClawL.setPosition(-1);
            robot.ClawR.setPosition(1);
        }
        if (gamepad1.b) {
            robot.ClawL.setPosition(1);
            robot.ClawR.setPosition(-1);
        }
        if (gamepad1.x) {
            robot.ClawL.setPosition(0);
            robot.ClawR.setPosition(0);
        }

        robot.Left.setPower(speed - turn);
        robot.Right.setPower(speed + turn);

        telemetry.addData("Speed ",speed);
        telemetry.addData("Turn Value ",turn);
        telemetry.update();
    }


}
