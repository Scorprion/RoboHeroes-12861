package TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import TestBot.Init.HardwareTestBot;

@TeleOp(name="Empty Chassis Drive", group="EmptyChassis")
public class EmptyChassisDrive extends OpMode {
    double speed = 0;
    double turn = 0;
    protected HardwareTestBot robot = new HardwareTestBot();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){

            turn = gamepad1.left_stick_x;
            speed = gamepad1.left_stick_y;

            if (turn < -0.1 || turn > 0.1) {
                robot.frontright.setPower(-turn);
                robot.frontleft.setPower(-turn);
                robot.backright.setPower(turn);
                robot.backleft.setPower(turn);
            }

            robot.frontright.setPower(speed);
            robot.frontleft.setPower(speed);
            robot.backright.setPower(speed);
            robot.backleft.setPower(speed);

    }

}
