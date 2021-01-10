package org.firstinspires.ftc.teamcode.Hermes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.KalmanFilter;

import static org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated.*;
import static org.firstinspires.ftc.teamcode.Hermes.HermesConstants.*;


@TeleOp(group = "Hermes")
public class FullLocalization extends OpMode {

    private double turnspeed, strafespeed, speed;


    // Localization with encoders
    private double xEnc = 0, yEnc = 0, x_prev = 0, y_prev = 0;

    // Localizations with powers/motor inputs
    private volatile double x_veloc, y_veloc, deltaTime = 0;
    private double motorV1 = 0, motorV2 = 0, motorV3 = 0, motorV4 = 0;  // Motor velocities (more accurately the powers though) labelled the same way as quadrants on the cartesian plane
    private ElapsedTime timer = new ElapsedTime();

    // Weight Average Parameter - weight (percentage) given to the encoders compared to the motor speed prediction method
    private final double weight = 0.75;

    HardwareHermes robot = new HardwareHermes();

    PositionPrediction backgroundTracker = new PositionPrediction();
    Thread thread = new Thread(backgroundTracker, "PosPred");

    private RealVector X = new ArrayRealVector(new double[] {0., 0., 0., 0.});  // Initial state {X, Y, Xdot, Ydot}
    private RealMatrix P = MatrixUtils.createRealIdentityMatrix(4);

    private RealMatrix Q = MatrixUtils.createRealIdentityMatrix(4);  // This could be updated based on the input values or something, but to keep it simple, it's just an identity matrix for now

    volatile KalmanFilter filter = new KalmanFilter(X, P);

    FtcDashboard dashboard = FtcDashboard.getInstance();

    class PositionPrediction implements Runnable {
        private boolean stopRequested = false;
        private double currentDheading = 0;  // Current angle in degrees
        private double currentHeading = 0;  // Current angle in radians
        private double previousTime = 0, deltaAngle = 0, previousAngle = 0;


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
            while(!this.isStopRequested()) {
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

                /*
                List<Integer> pos = Arrays.asList(robot.FrontRight.getCurrentPosition(), robot.FrontLeft.getCurrentPosition(), robot.BackLeft.getCurrentPosition(), robot.BackRight.getCurrentPosition());
                xEnc = AVG_RAD * (-pos.get(0) + pos.get(1) + -pos.get(2) + pos.get(3));
                yEnc = AVG_RAD * (pos.stream().mapToInt(Integer::intValue).sum());
                */


                x_veloc = highPassFilter(AVG_RAD *
                        (-robot.FrontRight.getVelocity() +
                         robot.FrontLeft.getVelocity() +
                         -robot.BackLeft.getVelocity() +
                         robot.BackRight.getVelocity()), 1e-3);

                y_veloc = highPassFilter(AVG_RAD *
                        (robot.FrontRight.getVelocity() +
                                robot.FrontLeft.getVelocity() +
                                robot.BackLeft.getVelocity() +
                                robot.BackRight.getVelocity()), 1e-2);


                deltaTime = timer.seconds() - previousTime;  // Update deltaTime between calculations
                double[] newVeloc = rotate(new double[] {x_veloc, y_veloc}, new double[] {0, 0}, this.currentHeading);
                xEnc += newVeloc[0] * deltaTime;
                yEnc += newVeloc[1] * deltaTime;

                RealVector Z = new ArrayRealVector(new double[] {xEnc, yEnc, newVeloc[0], newVeloc[1]});

                // Only using Q here to avoid creating another identity matrix (has no other meaning)
                filter.update(Z, Q, Q);

                update_heading(robot.imu.getAngularOrientation().firstAngle);

                previousTime = timer.seconds();  // Update previous time
            }
        }
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        dashboard.setTelemetryTransmissionInterval(25);
        thread.start();
        // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
    }

    @Override
    public void stop() {
        backgroundTracker.requestStop();
        backgroundTracker.reset();
    }

    public void drawRobot(Canvas can, double x, double y, double heading) {
        can.strokeCircle(x, y, RADIUS_X);

        double[] direcLine1 = new double[] {x, y};
        double[] direcLine2 = new double[] {x + RADIUS_X * Math.cos(Math.toRadians(heading)), y + RADIUS_Y * Math.sin(Math.toRadians(heading))};

        can.strokeLine(direcLine1[0], direcLine1[1], direcLine2[0], direcLine2[1]);
    }

    private double highPassFilter(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStroke("#4CAF50");

        turnspeed = -gamepad1.right_stick_x * ROBOT_CONTROL_SPEED;
        strafespeed = gamepad1.left_stick_x * ROBOT_CONTROL_SPEED;
        speed = gamepad1.left_stick_y * -ROBOT_CONTROL_SPEED;

        motorV1 = speed - strafespeed - turnspeed;
        motorV2 = speed + strafespeed + turnspeed;
        motorV3 = speed - strafespeed + turnspeed;
        motorV4 = speed + strafespeed - turnspeed;


        // Strafing
        robot.FrontRight.setPower(motorV1);
        robot.FrontLeft.setPower(motorV2);
        robot.BackLeft.setPower(motorV3);
        robot.BackRight.setPower(motorV4);


        // ------- Telemetry -------
        telemetry.addData("Delta time", deltaTime);

        telemetry.addData("X-pos Enc", xEnc);
        telemetry.addData("Y-pos Enc", yEnc);

        telemetry.addData("Kalman estimate", filter.get_state());

        telemetry.addData("Angle", backgroundTracker.currentDheading);
        telemetry.update();

        RealVector pos = filter.get_state();


        drawRobot(fieldOverlay, pos.getEntry(1), -pos.getEntry(0), backgroundTracker.currentDheading);
        packet.put("Kalman X", pos.getEntry(0));
        packet.put("Kalman Y", pos.getEntry(1));
        packet.put("Kalman Xdot", pos.getEntry(2));
        packet.put("Kalman Ydot", pos.getEntry(3));

        packet.put("Enc X", xEnc);
        packet.put("Enc Y", yEnc);
        packet.put("Heading", backgroundTracker.currentDheading);

        dashboard.sendTelemetryPacket(packet);
    }
}
