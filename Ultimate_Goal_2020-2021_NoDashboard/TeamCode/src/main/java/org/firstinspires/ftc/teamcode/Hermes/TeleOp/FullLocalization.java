package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.KalmanFilter;
import org.firstinspires.ftc.teamcode.PID;

@TeleOp(name = "KalmanFilter", group = "Hermes")
public class FullLocalization extends OpMode {

    private double turnspeed, strafespeed, speed;
    private final double robotControlSpeed = 0.7;

    // Localization with encoders
    private double x_enc = 0, y_enc = 0, x_prev = 0, y_prev = 0;
    private double w1, w2, w3, w4;

    // Localizations with powers/motor inputs
    private volatile double x_veloc, y_veloc, previous_time = 0, delta_time;
    private double motor_v1 = 0, motor_v2 = 0, motor_v3 = 0, motor_v4 = 0;  // Motor velocities (more accurately the powers though) labelled the same way as quadrants on the cartesian plane
    private ElapsedTime timer = new ElapsedTime();

    // Weight Average Parameter - weight (percentage) given to the encoders compared to the motor speed prediction method
    private final double weight = 0.75;

    final double wheel_radius = Math.PI * 75 * 5 / 127;  // wheel radius (inch)
    final double countsPerInch = 560 * (1 / wheel_radius); // 560 counts per rev
    final double wheel_rad_per_second = 3.042 * wheel_radius * robotControlSpeed;   // (300 is theoretical max) assuming 182.5 is the max load rpm of the motors (based on estimate) -> 3.042 rps

    final double avg_rad = 1 / (4 * countsPerInch);  // Added the division of counts per inch to avoid the extra calculation


    HardwareHermes robot = new HardwareHermes();
    PID angle_tracker = new PID(0, 0, 0, 0.0);
    PositionPrediction background_tracker = new PositionPrediction();
    Thread thread = new Thread(background_tracker, "PosPred");


    private RealVector X = new ArrayRealVector(new double[] {0., 0., 0., 0.});  // Initial state {X, Y, Xdot, Ydot}
    private RealMatrix P = MatrixUtils.createRealIdentityMatrix(4);

    private RealMatrix Q = MatrixUtils.createRealIdentityMatrix(4);  // This could be updated based on the input values or something, but to keep it simple, it's just an identity matrix for now

    volatile KalmanFilter filter = new KalmanFilter(X, P);

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
            while(!isStopRequested()) {
                // Position prediction
                delta_time = timer.seconds() - previous_time;
                RealMatrix A = MatrixUtils.createRealMatrix(new double[][] {{1., 0., delta_time, 0.},
                                                                            {0., 1., 0., delta_time},
                                                                            {0., 0., 1., 0.},
                                                                            {0., 0., 0., 1.}});
                RealMatrix B = MatrixUtils.createRealMatrix(new double[][] {{-1., 1., -1., 1.},
                                                                            {1., 1., 1., 1.},
                                                                            {-1., 1., -1., 1.},
                                                                            {1., 1., 1., 1.}});
                B.transpose().operate(new double[] {
                                        wheel_rad_per_second * delta_time,
                                        wheel_rad_per_second * delta_time,
                                        wheel_rad_per_second,
                                        wheel_rad_per_second
                });

                RealVector U = new ArrayRealVector(new double[] {motor_v1, motor_v2, motor_v3, motor_v4});

                //  Can't transpose after the operation for B?
                filter.predict(A, B.transpose(), U, Q);


                w1 = robot.FrontRight.getCurrentPosition();
                w2 = robot.FrontLeft.getCurrentPosition();
                w3 = robot.BackLeft.getCurrentPosition();
                w4 = robot.BackRight.getCurrentPosition();

                x_enc = avg_rad * (-w1 + w2 + -w3 + w4);
                y_enc = avg_rad * (w1 + w2 + w3 + w4);

                x_veloc = 0.25 * (1 / countsPerInch) *
                        (-robot.FrontRight.getVelocity() +
                         robot.FrontLeft.getVelocity() +
                         -robot.BackLeft.getVelocity() +
                         robot.BackRight.getVelocity());

                y_veloc = 0.25 * (1 / countsPerInch) *
                        (robot.FrontRight.getVelocity() +
                         robot.FrontLeft.getVelocity() +
                         robot.BackLeft.getVelocity() +
                         robot.BackRight.getVelocity());

                RealVector Z = new ArrayRealVector(new double[] {x_enc, y_enc, x_veloc, y_veloc});

                // Only using Q here to avoid creating another identity matrix (has no other meaning)
                filter.update(Z, Q, Q);
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
        turnspeed = -gamepad1.right_stick_x * robotControlSpeed;
        strafespeed = gamepad1.left_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * -robotControlSpeed;

        motor_v1 = speed - strafespeed - turnspeed;
        motor_v2 = speed + strafespeed + turnspeed;
        motor_v3 = speed - strafespeed + turnspeed;
        motor_v4 = speed + strafespeed - turnspeed;


        // Strafing
        robot.FrontRight.setPower(motor_v1);
        robot.FrontLeft.setPower(motor_v2);
        robot.BackLeft.setPower(motor_v3);
        robot.BackRight.setPower(motor_v4);


        // ------- Telemetry -------
        telemetry.addData("Delta time", delta_time);

        telemetry.addData("X-pos Enc", x_enc);
        telemetry.addData("Y-pos Enc", y_enc);

        telemetry.addData("Kalman estimate", filter.get_state());

        telemetry.addData("Angle", angle_tracker.likeallelse(robot.imu.getAngularOrientation().firstAngle));
        telemetry.update();
    }
}
