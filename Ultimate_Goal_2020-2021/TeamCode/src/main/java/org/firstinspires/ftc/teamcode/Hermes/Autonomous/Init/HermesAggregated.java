package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.KalmanFilter;
import org.firstinspires.ftc.teamcode.PID;

import java.math.BigDecimal;
import java.math.RoundingMode;

@SuppressWarnings({"WeakerAccess", "SameParameterValue"})
public class HermesAggregated extends LinearOpMode {
    final double wheel_radius = Math.PI * 75 * 5 / 127;  // wheel radius (inch)
    public final double countsPerInch = 560 * (1 / wheel_radius); // 560 counts per rev
    public HardwareHermes robot = new HardwareHermes();

    // Localization
    HermesAggregated.PositionPrediction background_tracker = new HermesAggregated.PositionPrediction();
    Thread thread = new Thread(background_tracker, "PosPred");

    private RealVector X = new ArrayRealVector(new double[] {0., 0., 0., 0.});  // Initial state {X, Y, Xdot, Ydot}
    private RealMatrix P = MatrixUtils.createRealIdentityMatrix(4);
    private RealMatrix Q = MatrixUtils.createRealIdentityMatrix(4);  // This could be updated based on the input values or something, but to keep it simple, it's just an identity matrix for now
    volatile KalmanFilter filter = new KalmanFilter(X, P);

    private ElapsedTime timer = new ElapsedTime();
    public double motor_v1, motor_v2, motor_v3, motor_v4;  // Velocities of the motors
    public volatile double x_enc, y_enc, x_veloc, y_veloc;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        thread.start();
    }

    private class PositionPrediction implements Runnable {
        private boolean stopRequested = false;
        private double current_dheading = 0;  // Current angle in degrees
        private double current_heading = 0;  // Current angle in radians
        private double previous_time = 0, delta_time = 0, delta_angle = 0, previous_angle = 0;
        private final double wheel_rad_per_second = 3.042 * wheel_radius;
        private final double avg_rad = 1 / (4 * countsPerInch);  // Added the division of counts per inch to avoid the extra calculation

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


                double w1 = robot.FrontRight.getCurrentPosition();
                double w2 = robot.FrontLeft.getCurrentPosition();
                double w3 = robot.BackLeft.getCurrentPosition();
                double w4 = robot.BackRight.getCurrentPosition();

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

                update_heading(robot.imu.getAngularOrientation().firstAngle);

                previous_time = timer.seconds();  // Update previous time
            }
        }
    }

    public void runTo(double x, double y, double angle, double[] xPIDCoeffs, double[] yPIDCoeffs, double[] angPIDCoeffs) {
        PID xpid = new PID(xPIDCoeffs[0], xPIDCoeffs[1], xPIDCoeffs[2], 0.3);
        PID ypid = new PID(yPIDCoeffs[0], yPIDCoeffs[1], yPIDCoeffs[2], 0.3);
        PID angpid = new PID(angPIDCoeffs[0], angPIDCoeffs[1], angPIDCoeffs[2], 0.3);

        while(opModeIsActive()) {
            RealVector position = filter.get_state();

            double strafe = PID.constrain(xpid.getPID(x - position.getEntry(0)), -1./3., 1./3.);
            double forward = PID.constrain(ypid.getPID(y - position.getEntry(1)), -1./3., 1./3.);
            double turn = PID.constrain(angpid.getPID(angle - background_tracker.current_dheading), -1./3., 1./3.);

            robot.FrontRight.setPower(forward - strafe - turn);
            robot.BackRight.setPower(forward + strafe - turn);
            robot.FrontLeft.setPower(forward + strafe + turn);
            robot.BackLeft.setPower(forward - strafe + turn);

            telemetry.addData("X:", position.getEntry(0));
            telemetry.addData("Y", position.getEntry(1));
            telemetry.addData("Heading", background_tracker.current_dheading);
            telemetry.addData("Outputs:", "%.2f, %.2f, %.2f", (forward), (strafe), (turn));
            telemetry.update();
        }
    }

    public double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_DOWN);
        return bd.doubleValue();
    }

    public boolean inrange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public double likeallelse(double angle) {
        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;

        return angle;
    }
}

