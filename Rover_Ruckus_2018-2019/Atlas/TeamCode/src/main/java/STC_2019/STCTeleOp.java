package STC_2019;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "STC_TeleOp", group= "Pushbot")
public class STCTeleOp extends OpMode {

    private double speed = 0;
    private double turnspeed = 0;
    private double clampspeed = 0;
    private double liftspeed = 0;
    HardwareSTC robot = new HardwareSTC();


    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        speed = gamepad1.left_stick_y;
        turnspeed = gamepad1.right_stick_x;

        liftspeed = gamepad2.right_stick_y;
        clampspeed = gamepad2.left_stick_y;

        //Gamepad 1
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= 0.1) {
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);
        }

        if(gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
            robot.Left.setPower(-turnspeed);
            robot.Right.setPower(turnspeed);
        }

        if(gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1) {
            robot.Lift.setPower(liftspeed);
        } else {
            robot.Lift.setPower(0);
        }

        if(gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1) {
            robot.Clamp.setPower(clampspeed);
        } else {
            robot.Clamp.setPower(0);
        }

    }
}
