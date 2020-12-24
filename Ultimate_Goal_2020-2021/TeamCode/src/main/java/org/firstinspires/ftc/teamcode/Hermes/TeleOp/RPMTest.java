package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;

import static java.lang.Thread.sleep;

@TeleOp(name= "RPM Calc", group= "Pushbot")
public class RPMTest extends OpMode {
    private double robotControlSpeed = 0.7;

    private final double revs_per_count = 3.0 / 28.0;

    private ElapsedTime timer = new ElapsedTime();
    // private double prev_pos1 = 0, prev_pos2 = 0, prev_pos3 = 0, prev_pos4 = 0;
    private volatile double current_time = 0;


    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;
    private double moving_avg = 0, N = 1;

    HardwareHermes robot = new HardwareHermes();

    @Override
    public void init() {
        robot.init(hardwareMap);
        moving_avg = robot.FrontRight.getVelocity();
    }


    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 1         |
        |                           |
        |                           |
        -----------------------------
         */


        //Strafing
        robot.FrontRight.setPower(speed - strafespeed - turnspeed);
        robot.BackRight.setPower(speed + strafespeed - turnspeed);
        robot.FrontLeft.setPower(speed + strafespeed + turnspeed);
        robot.BackLeft.setPower(speed - strafespeed + turnspeed);

        telemetry.addData("RPM Estimates: ", "%.5f, %.5f, %.5f, %.5f", (robot.FrontRight.getVelocity(AngleUnit.DEGREES) / 60),
                (robot.FrontLeft.getVelocity(AngleUnit.DEGREES) / 60),
                (robot.BackLeft.getVelocity(AngleUnit.DEGREES) / 60),
                (robot.BackRight.getVelocity(AngleUnit.DEGREES) / 60));

        N += 1;
        moving_avg += (robot.FrontRight.getVelocity() - moving_avg) / N;

        telemetry.addData("Veloc estimate", "%.5f", moving_avg);


        telemetry.addData("Encoder1", robot.FrontRight.getCurrentPosition());
        telemetry.addData("Time: ", current_time);
        telemetry.update();
    }

}
