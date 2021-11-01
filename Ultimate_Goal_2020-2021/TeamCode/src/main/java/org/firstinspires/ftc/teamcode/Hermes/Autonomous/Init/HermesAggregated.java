package org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Profiles.TriangleProfile;
import org.firstinspires.ftc.teamcode.KalmanFilter;

import java.math.BigDecimal;
import java.math.RoundingMode;

import static org.firstinspires.ftc.teamcode.Hermes.HermesConstants.*;

@SuppressWarnings({"WeakerAccess", "SameParameterValue"})
public class HermesAggregated extends LinearOpMode {
    public HardwareHermes robot = new HardwareHermes();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Localization
    public HermesAggregated.PositionPrediction backgroundTracker = new HermesAggregated.PositionPrediction();
    Thread thread = new Thread(backgroundTracker, "PosPred");

    private RealVector X = new ArrayRealVector(new double[] {0., 0., 0., 0.});  // Initial state {X, Y, Xdot, Ydot}
    private RealMatrix P = MatrixUtils.createRealIdentityMatrix(4);
    private RealMatrix Q = MatrixUtils.createRealIdentityMatrix(4);  // This could be updated based on the input values or something, but to keep it simple, it's just an identity matrix for now
    volatile KalmanFilter filter = new KalmanFilter(X, P);

    private ElapsedTime timer = new ElapsedTime();
    public double motorV1, motorV2, motorV3, motorV4;  // Velocities of the motors
    public volatile double xEnc, yEnc, xVeloc, yVeloc;

    ElapsedTime moveTimer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();

        thread.start();
    }

    public class PositionPrediction implements Runnable {
        private boolean stopRequested = false;
        private double currentDheading = 0;  // Current angle in degrees
        private double currentHeading = 0;  // Current angle in radians
        private double previousTime = 0, deltaTime = 0, deltaAngle = 0, previousAngle = 0;
        private double angVeloc = 0;


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

        public void reset() {
            this.currentDheading = 0;
            this.previousAngle = 0;
            previousTime = 0;

            for(DcMotorEx motor : robot.motorList) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            timer.reset();
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

                RealVector U = new ArrayRealVector(new double[] {robot.FrontRight.getPower(),
                        robot.FrontLeft.getPower(),
                        robot.BackLeft.getPower(),
                        robot.BackRight.getPower()});

                //  Can't transpose after the operation for B?
                filter.predict(A, B.transpose(), U, Q);


                xVeloc = AVG_RAD *
                        (-robot.FrontRight.getVelocity() +
                                robot.FrontLeft.getVelocity() +
                                -robot.BackLeft.getVelocity() +
                                robot.BackRight.getVelocity());

                yVeloc = AVG_RAD *
                        (robot.FrontRight.getVelocity() +
                                robot.FrontLeft.getVelocity() +
                                robot.BackLeft.getVelocity() +
                                robot.BackRight.getVelocity());

                deltaTime = timer.seconds() - previousTime;  // Update deltaTime between calculations
                double[] newVeloc = rotate(new double[] {xVeloc, yVeloc}, new double[] {0, 0}, this.currentHeading);
                xEnc += newVeloc[0] * deltaTime;
                yEnc += newVeloc[1] * deltaTime;

                RealVector Z = new ArrayRealVector(new double[] {xEnc, yEnc, xVeloc, yVeloc});

                // Only using Q here to avoid creating another identity matrix (has no other meaning)
                filter.update(Z, Q, Q);

                update_heading(robot.imu.getAngularOrientation().firstAngle);

                previousTime = timer.seconds();  // Update previous time
                this.angVeloc = Math.toDegrees(AVG_RAD * (1. / 6.347) * (-robot.FrontRight.getVelocity() + robot.FrontLeft.getVelocity() + robot.BackLeft.getVelocity() - robot.BackRight.getVelocity()));
            }
        }
    }

    public void runTo(double x, double y, double angle, double allotedSec) {
        RealVector position = filter.get_state();

        TriangleProfile xProfile = new TriangleProfile(x - position.getEntry(0), allotedSec);
        TriangleProfile yProfile = new TriangleProfile(y - position.getEntry(1), allotedSec);
        TriangleProfile angProfile = new TriangleProfile(angle - backgroundTracker.currentDheading, allotedSec);


        moveTimer.reset();
        while(opModeIsActive() && allotedSec > moveTimer.seconds()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStroke("#4CAF50");

            position = filter.get_state();

            double strafe = xProfile.calculate(moveTimer.seconds()) * COUNTS_PER_INCH;
            double forward = yProfile.calculate(moveTimer.seconds()) * COUNTS_PER_INCH;
            double turn = angProfile.calculate(moveTimer.seconds()) * COUNTS_PER_INCH;

            robot.FrontRight.setVelocity(forward - strafe - turn);
            robot.BackRight.setVelocity(forward + strafe - turn);
            robot.FrontLeft.setVelocity(forward + strafe + turn);
            robot.BackLeft.setVelocity(forward - strafe + turn);

            // Swapped x and y to fit intuition
            drawRobot(fieldOverlay, position.getEntry(1), -position.getEntry(0), backgroundTracker.currentDheading);

            packet.put("X", robot.BackRight.getVelocity());
            packet.put("X setpoint", strafe);
            packet.put("X error", x - position.getEntry(0));
            packet.put("X position", position.getEntry(0));


            packet.put("Y", position.getEntry(3));
            packet.put("Y setpoint", forward);
            packet.put("Y error", forward - position.getEntry(3));
            packet.put("Heading", backgroundTracker.angVeloc);
            packet.put("Heading setpoint", turn);
            packet.put("Heading error", turn - backgroundTracker.angVeloc);

            packet.put("Time", moveTimer.seconds());

            packet.put("Forward output", forward);
            packet.put("Strafe output", strafe);
            packet.put("Turn output", turn);
            // packet.putAll("Outputs:", "%.2f, %.2f, %.2f", (forward), (strafe), (turn));

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public static void drawRobot(Canvas can, double x, double y, double heading) {
        can.strokeCircle(x, y, RADIUS_X);

        double[] direcLine1 = new double[] {x, y};
        double[] direcLine2 = new double[] {x + RADIUS_X * Math.cos(Math.toRadians(heading)), y + RADIUS_Y * Math.sin(Math.toRadians(heading))};

        can.strokeLine(direcLine1[0], direcLine1[1], direcLine2[0], direcLine2[1]);
    }

    public static double[] rotate(double[] toRotate, double[] around, double radAngle) {
        double[] rotation = new double[toRotate.length];

        // Shift, rotate, shift back
        rotation[0] = -around[0] + toRotate[0] * Math.cos(radAngle) + toRotate[1] * Math.sin(radAngle) + around[0];
        rotation[1] = -around[1] + toRotate[0] * -Math.sin(radAngle) + toRotate[1] * Math.cos(radAngle) + around[1];
        return rotation;
    }

    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_DOWN);
        return bd.doubleValue();
    }

    public static boolean inrange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public static double likeallelse(double angle) {
        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;

        return angle;
    }
}

