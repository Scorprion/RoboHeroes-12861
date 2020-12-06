package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;

import static java.lang.Thread.sleep;

@TeleOp(name= "RPM Calc", group= "Pushbot")
public class RPMTest extends OpMode {
    private double robotControlSpeed = 0.7;

    private final double revs_per_count = 3.0 / 28.0;

    private ElapsedTime timer = new ElapsedTime();
    private volatile double rps1 = 0, rps2 = 0, rps3 = 0, rps4 = 0;
    // private double prev_pos1 = 0, prev_pos2 = 0, prev_pos3 = 0, prev_pos4 = 0;
    private volatile double current_time = 0;


    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;

    CalcRPM background_tracker = new CalcRPM();
    Thread thread = new Thread(background_tracker, "RPMCalc");

    HardwareHermes robot = new HardwareHermes();

    @Override
    public void init() {
        robot.init(hardwareMap);
        thread.start();
    }

    private class CalcRPM implements Runnable {
        private boolean stopRequested = false;
        public synchronized void requestStop() {
            this.stopRequested = true;
        }

        public synchronized boolean isStopRequested() {
            return this.stopRequested;
        }

        @Override
        public void run() {
            while(!isStopRequested()) {
                current_time = timer.seconds();
                rps1 = (robot.FrontRight.getCurrentPosition() / current_time) * revs_per_count;
                rps2 = (robot.FrontLeft.getCurrentPosition() / current_time) * revs_per_count;
                rps3 = (robot.BackLeft.getCurrentPosition() / current_time) * revs_per_count;
                rps4 = (robot.BackRight.getCurrentPosition() / current_time) * revs_per_count;
            }
        }
    }

    @Override
    public void stop() {
        background_tracker.requestStop();
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

        telemetry.addData("RPM Estimates: ", "%.5f, %.5f, %.5f, %.5f", (rps1), (rps2), (rps3), (rps4));
        telemetry.addData("Encoder1", robot.FrontRight.getCurrentPosition());
        telemetry.addData("Time: ", current_time);
        telemetry.update();
    }

}
