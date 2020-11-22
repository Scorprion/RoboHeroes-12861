package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.PID;

@TeleOp(name = "HermesMovement", group = "Hermes")
public class HermesMovement extends OpMode {
    
    private double x_pos = 0;
    private double y_pos = 0;
    private double theta = 0;

    private double turnspeed, strafespeed, speed;
    private final double robotControlSpeed = 0.7;

    private double w1, w2, w3, w4;

    // 80 * 0.0394 to convert 80 mm to inches
    final double wheel_radius = 2 * Math.PI * (80 * 0.0394);  // wheel radius (inch)
    final double countsPerInch = 54.722;  // 51.7444455?
    final double l_x = 13;  // wheel distance x-wise (inch)
    final double l_y = 11.25;  // wheel distance y-wise (inch)


    final double avg_rad = wheel_radius / 4;
    final double inverse_sum_dist = 1 / (l_x + l_y);


    HardwareHermes robot = new HardwareHermes();
    PID angle_tracker = new PID(0, 0, 0, 0.0);



    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        w1 = robot.FrontRight.getCurrentPosition() / countsPerInch;
        w2 = robot.FrontLeft.getCurrentPosition() / countsPerInch;
        w3 = robot.BackLeft.getCurrentPosition() / countsPerInch;
        w4 = robot.BackRight.getCurrentPosition() / countsPerInch;

        y_pos = avg_rad * (w1 + w2 + w3 + w4);
        x_pos = avg_rad * (-w1 + w2 + -w3 + w4);
        theta = avg_rad * (inverse_sum_dist * -w1 + inverse_sum_dist * w2 + inverse_sum_dist * w3 + inverse_sum_dist * -w4);

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

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1);

        telemetry.addData("X-pos Enc", x_pos);
        telemetry.addData("Y-pos Enc", y_pos);
        telemetry.addData("IMU", robot.imu.getPosition());
        telemetry.addData("Angle", theta * 180 / Math.PI);
        telemetry.addData("Real Angle", angle_tracker.likeallelse(robot.imu.getAngularOrientation().firstAngle));

        telemetry.addData("Distance Sensor (inch):", robot.ds.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance Sensor (mm):", robot.ds.getDistance(DistanceUnit.MM));

        telemetry.addData("Average right", (robot.BackRight.getCurrentPosition() + robot.FrontRight.getCurrentPosition()) / 2);
        telemetry.addData("Average left", (robot.BackLeft.getCurrentPosition() + robot.FrontLeft.getCurrentPosition()) / 2);

        if(robot.ds.getDistance(DistanceUnit.MM) < 60) {
            telemetry.addLine("4 rings");
        }
        else if(robot.ds.getDistance(DistanceUnit.MM) < 100) {
            telemetry.addLine("1 ring");
        } else {
            telemetry.addLine("No rings");
        }

        telemetry.update();
    }
}
