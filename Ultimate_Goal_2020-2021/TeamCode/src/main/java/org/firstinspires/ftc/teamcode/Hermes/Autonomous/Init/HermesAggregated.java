package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.KalmanFilter;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.PIDCoeffs;

import java.math.BigDecimal;
import java.math.RoundingMode;

import static org.firstinspires.ftc.teamcode.Hermes.HermesConstants.*;

@SuppressWarnings({"WeakerAccess", "SameParameterValue"})
public class HermesAggregated extends LinearOpMode {
    public HardwareHermes robot = new HardwareHermes();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();
    Canvas fieldOverlay = packet.fieldOverlay();

    // Localization
    HermesAggregated.PositionPrediction backgroundTracker = new HermesAggregated.PositionPrediction();
    Thread thread = new Thread(backgroundTracker, "PosPred");

    private RealVector X = new ArrayRealVector(new double[] {0., 0., 0., 0.});  // Initial state {X, Y, Xdot, Ydot}
    private RealMatrix P = MatrixUtils.createRealIdentityMatrix(4);
    private RealMatrix Q = MatrixUtils.createRealIdentityMatrix(4);  // This could be updated based on the input values or something, but to keep it simple, it's just an identity matrix for now
    volatile KalmanFilter filter = new KalmanFilter(X, P);

    private ElapsedTime timer = new ElapsedTime();
    public double motorV1, motorV2, motorV3, motorV4;  // Velocities of the motors
    public volatile double x_enc, y_enc, x_veloc, y_veloc;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        dashboard.setTelemetryTransmissionInterval(25);
        fieldOverlay.setStroke("#4CAF50");

        waitForStart();

        thread.start();
    }

    private class PositionPrediction implements Runnable {
        private boolean stopRequested = false;
        private double currentDheading = 0;  // Current angle in degrees
        private double currentHeading = 0;  // Current angle in radians
        private double previousTime = 0, deltaTime = 0, deltaAngle = 0, previousAngle = 0;


        public synchronized void requestStop() {
            this.stopRequested = true;
        }

        public synchronized boolean isStopRequested() {
            return this.stopRequested;
        }


        public synchronized void update_heading(double angle) {
            this.deltaAngle = angle - this.previousAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            this.currentDheading += deltaAngle;
            this.currentHeading = Math.toRadians(this.currentDheading);
            this.previousAngle = angle;
        }


        @Override
        public void run() {
            while(opModeIsActive()) {
                // Position prediction
                deltaTime = timer.seconds() - previousTime;
                RealMatrix A = MatrixUtils.createRealMatrix(new double[][] {{1., 0., deltaTime, 0.},
                        {0., 1., 0., deltaTime},
                        {0., 0., 1., 0.},
                        {0., 0., 0., 1.}});
                RealMatrix B = MatrixUtils.createRealMatrix(new double[][] {{-1., 1., -1., 1.},
                        {1., 1., 1., 1.},
                        {-1., 1., -1., 1.},
                        {1., 1., 1., 1.}});
                B.transpose().operate(new double[] {
                        WHEEL_RAD_PER_SECOND * deltaTime,
                        WHEEL_RAD_PER_SECOND * deltaTime,
                        WHEEL_RAD_PER_SECOND,
                        WHEEL_RAD_PER_SECOND
                });

                RealVector U = new ArrayRealVector(new double[] {motorV1, motorV2, motorV3, motorV4});

                //  Can't transpose after the operation for B?
                filter.predict(A, B.transpose(), U, Q);


                double w1 = robot.FrontRight.getCurrentPosition();
                double w2 = robot.FrontLeft.getCurrentPosition();
                double w3 = robot.BackLeft.getCurrentPosition();
                double w4 = robot.BackRight.getCurrentPosition();

                x_enc = AVG_RAD * (-w1 + w2 + -w3 + w4);
                y_enc = AVG_RAD * (w1 + w2 + w3 + w4);

                x_veloc = 0.25 * (1 / COUNTS_PER_INCH) *
                        (-robot.FrontRight.getVelocity() +
                                robot.FrontLeft.getVelocity() +
                                -robot.BackLeft.getVelocity() +
                                robot.BackRight.getVelocity());

                y_veloc = 0.25 * (1 / COUNTS_PER_INCH) *
                        (robot.FrontRight.getVelocity() +
                                robot.FrontLeft.getVelocity() +
                                robot.BackLeft.getVelocity() +
                                robot.BackRight.getVelocity());

                RealVector Z = new ArrayRealVector(new double[] {x_enc, y_enc, x_veloc, y_veloc});

                // Only using Q here to avoid creating another identity matrix (has no other meaning)
                filter.update(Z, Q, Q);

                update_heading(robot.imu.getAngularOrientation().firstAngle);

                previousTime = timer.seconds();  // Update previous time
            }
        }
    }

    public void runTo(double x, double y, double angle, double allotedSec, PIDCoeffs xPIDCoeffs, PIDCoeffs yPIDCoeffs, PIDCoeffs angPIDCoeffs) {
        ElapsedTime timer = new ElapsedTime();

        PID xpid = new PID(xPIDCoeffs.getP(), xPIDCoeffs.getI(), xPIDCoeffs.getD(),0.3);
        PID ypid = new PID(yPIDCoeffs.getP(), yPIDCoeffs.getI(), yPIDCoeffs.getD(),0.3);
        PID angpid = new PID(angPIDCoeffs.getP(), angPIDCoeffs.getI(), angPIDCoeffs.getD(),0.3);

        timer.reset();
        while(opModeIsActive() && allotedSec < timer.seconds()) {
            RealVector position = filter.get_state();

            double strafe = PID.constrain(xpid.getPID((x - position.getEntry(0)) / 144), -1./3., 1./3.);
            double forward = PID.constrain(ypid.getPID((y - position.getEntry(1)) / 144), -1./3., 1./3.);
            double turn = PID.constrain(angpid.getPID((angle - backgroundTracker.currentDheading) / 360), -1./3., 1./3.);

            robot.FrontRight.setPower(forward - strafe - turn);
            robot.BackRight.setPower(forward + strafe - turn);
            robot.FrontLeft.setPower(forward + strafe + turn);
            robot.BackLeft.setPower(forward - strafe + turn);

            drawRobot(fieldOverlay, position.getEntry(0), position.getEntry(1), backgroundTracker.currentDheading);

            telemetry.addData("X:", position.getEntry(0));
            telemetry.addData("Y", position.getEntry(1));
            telemetry.addData("Heading", backgroundTracker.currentDheading);
            telemetry.addData("Outputs:", "%.2f, %.2f, %.2f", (forward), (strafe), (turn));
            telemetry.update();
        }
    }

    public void drawRobot(Canvas can, double x, double y, double heading) {
        can.strokeCircle(x, y, RADIUS_X);

        double[] direcLine1 = new double[] {x, y};
        double[] direcLine2 = new double[] {x + RADIUS_X * Math.cos(Math.toRadians(heading)), y + RADIUS_Y * Math.sin(Math.toRadians(heading))};

        can.strokeLine(direcLine1[0], direcLine1[1], direcLine2[0], direcLine2[1]);
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

