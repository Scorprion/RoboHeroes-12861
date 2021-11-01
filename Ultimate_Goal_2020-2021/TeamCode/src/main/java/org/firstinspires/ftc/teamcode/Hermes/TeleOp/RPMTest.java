package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;

import static java.lang.Thread.sleep;

@TeleOp(name= "RPM Calc", group= "Pushbot")
public class RPMTest extends OpMode {

    private final double revs_per_count = 3.0 / 28.0;

    private ElapsedTime timer = new ElapsedTime();
    // private double prev_pos1 = 0, prev_pos2 = 0, prev_pos3 = 0, prev_pos4 = 0;
    private volatile double current_time = 0;

    private double turnspeed = 0;
    private double strafespeed = 0;
    private double speed = 0;
    private double moving_avg = 0;
    private double constantAvg = 0;

    HardwareHermes robot = new HardwareHermes();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        robot.init(hardwareMap);
        moving_avg = robot.FrontRight.getVelocity();

    }


    @Override
    public void loop() {
        turnspeed = -gamepad1.right_stick_x;
        strafespeed = gamepad1.left_stick_x;
        speed = -gamepad1.left_stick_y;

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

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM FR", robot.FrontRight.getVelocity());
        packet.put("RPM FL", robot.FrontLeft.getVelocity(AngleUnit.DEGREES) / 60);
        packet.put("RPM BL", robot.BackLeft.getVelocity(AngleUnit.DEGREES) / 60);
        packet.put("BackLeft Param", constantAvg);
        packet.put("RPM BR", robot.BackRight.getVelocity(AngleUnit.DEGREES) / 60);

        moving_avg = Math.abs(robot.FrontRight.getVelocity()) * 0.9 + moving_avg * 0.1;
        packet.put("RPM Est", robot.FrontRight.getCurrent(CurrentUnit.AMPS) * constantAvg);

        constantAvg = 0.05 * (robot.BackLeft.getVelocity() / 60) / (robot.BackLeft.getCurrent(CurrentUnit.AMPS) + 1e-8) + 0.95 * constantAvg;
        packet.put("FR AVG", moving_avg);
        dashboard.sendTelemetryPacket(packet);

        /*
        telemetry.addData("RPM Estimates: ", "%.5f, %.5f, %.5f, %.5f", (Math.abs(robot.FrontRight.getVelocity(AngleUnit.RADIANS))),
                (Math.abs(robot.FrontLeft.getVelocity(AngleUnit.RADIANS))),
                (Math.abs(robot.BackLeft.getVelocity(AngleUnit.DEGREES) / 60)),
                (Math.abs(robot.BackRight.getVelocity(AngleUnit.DEGREES) / 60)));
        */

        telemetry.addData("Veloc estimate", "%.5f", moving_avg);


        telemetry.addData("Encoder1", robot.FrontRight.getCurrentPosition());
        telemetry.addData("Time: ", current_time);
        telemetry.update();
    }

}
