package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.PID;

@TeleOp(name = "HermesMovement", group = "Hermes")
public class HermesMovement extends OpMode {

    private double turnspeed, strafespeed, speed;
    private final double robotControlSpeed = 0.7;

    // Localization with encoders
    private double x_enc = 0, y_enc = 0, theta = 0;
    private double w1, w2, w3, w4;

    // Localizations with powers/motor inputs
    private volatile double x_pred, y_pred, previous_time = 0, delta_time;
    private final double velocity_avg = 0;
    private double motor_v1 = 0, motor_v2 = 0, motor_v3 = 0, motor_v4 = 0;  // Motor velocities (more accurately the powers though) labelled the same way as quadrants on the cartesian plane
    private final ElapsedTime timer = new ElapsedTime();

    // Weight Average Parameter - weight (percentage) given to the encoders compared to the motor speed prediction method
    private final double weight = 0.75;

    final double wheel_radius = Math.PI * 75 * 5 / 127;  // wheel radius (inch)
    final double countsPerInch = 560 * (1 / wheel_radius); // 560 counts per rev
    final double wheel_rad_per_second = 3.042 * wheel_radius * robotControlSpeed;   // (300 is theoretical max) assuming 182.5 is the max load rpm of the motors -> 3.042 rps
    final double l_x = 13.5;  // wheel distance x-wise (inch)
    final double l_y = 11.45;  // wheel distance y-wise (inch)


    final double avg_rad = 1 / (4 * countsPerInch);  // Added the division of counts per inch to avoid the extra calculation
    final double inverse_sum_dist = 1 / (l_x + l_y);


    HardwareHermes robot = new HardwareHermes();
    PID angle_tracker = new PID(0, 0, 0, 0.0, 0.0);
    PositionPrediction background_tracker = new PositionPrediction();
    Thread thread = new Thread(background_tracker, "PosPred");

    private class PositionPrediction implements Runnable {
        private boolean stopRequested = false;
        private double current_dheading = 0;  // Current angle in degrees
        private double current_heading = 0;  // Current angle in radians
        private double delta_angle = 0, previous_angle = 0;  // Variables used to calculate the current angle from the imu

        public synchronized void requestStop() {
            this.stopRequested = true;
        }

        public synchronized boolean isStopRequested() {
            return this.stopRequested;
        }


        public synchronized void update_heading(double angle) {
            this.delta_angle = angle - this.previous_angle;

            if (delta_angle < -180)
                delta_angle += 360;
            else if (delta_angle > 180)
                delta_angle -= 360;

            this.current_dheading += delta_angle;
            this.current_heading = Math.toRadians(this.current_dheading);
            this.previous_angle = angle;
        }


        @Override
        public void run() {
            while (!isStopRequested()) {
                // Position prediction
                delta_time = timer.seconds() - previous_time;  // Current time - previous time

                x_pred += wheel_rad_per_second * (-motor_v1 + motor_v2 + -motor_v3 + motor_v4) * delta_time;
                y_pred += wheel_rad_per_second * (motor_v1 + motor_v2 + motor_v3 + motor_v4) * delta_time;
                previous_time = timer.seconds();  // Update previous time
            }
        }
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        thread.start();
        // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
    }

    @Override
    public void stop() {
        background_tracker.requestStop();
    }

    @Override
    public void loop() {
        w1 = robot.FrontRight.getCurrentPosition();
        w2 = robot.FrontLeft.getCurrentPosition();
        w3 = robot.BackLeft.getCurrentPosition();
        w4 = robot.BackRight.getCurrentPosition();

        x_enc = avg_rad * (-w1 + w2 + -w3 + w4);
        y_enc = avg_rad * (w1 + w2 + w3 + w4);
        theta = avg_rad * (inverse_sum_dist * -w1 + inverse_sum_dist * w2 + inverse_sum_dist * w3 + inverse_sum_dist * -w4);

        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        motor_v1 = speed - strafespeed - turnspeed;
        motor_v2 = speed + strafespeed + turnspeed;
        motor_v3 = speed - strafespeed + turnspeed;
        motor_v4 = speed + strafespeed - turnspeed;


        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 1         |
        |                           |
        |                           |
        -----------------------------
         */


        // Strafing
        robot.FrontRight.setPower(motor_v1);
        robot.FrontLeft.setPower(motor_v2);
        robot.BackLeft.setPower(motor_v3);
        robot.BackRight.setPower(motor_v4);


        // ------- Telemetry -------
        telemetry.addData("X-pos pred", x_pred);
        telemetry.addData("Y-pos pred", y_pred);
        telemetry.addData("Delta time", delta_time);

        telemetry.addData("X-pos Enc", x_enc);
        telemetry.addData("Y-pos Enc", y_enc);

        telemetry.addData("X-pos estimate", (x_enc * weight + x_pred * (1 - weight)));
        telemetry.addData("Y-pos estimate", (y_enc * weight + y_pred * (1 - weight)));

        telemetry.addData("Angle (rad)", theta);
        telemetry.addData("Angle (deg)", theta * 180 / Math.PI);
        telemetry.addData("Real Angle", angle_tracker.likeallelse(robot.imu.getAngularOrientation().firstAngle));

        telemetry.addData("Distance Sensor (inch):", robot.ds.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance Sensor (mm):", robot.ds.getDistance(DistanceUnit.MM));

        telemetry.addData("Average right", (robot.BackRight.getCurrentPosition() + robot.FrontRight.getCurrentPosition()) / 2);
        telemetry.addData("Average left", (robot.BackLeft.getCurrentPosition() + robot.FrontLeft.getCurrentPosition()) / 2);

        if (robot.ds.getDistance(DistanceUnit.MM) < 60) {
            telemetry.addLine("4 rings");
        } else if (robot.ds.getDistance(DistanceUnit.MM) < 100) {
            telemetry.addLine("1 ring");
        } else {
            telemetry.addLine("No rings");
        }

        telemetry.update();
    }
}
