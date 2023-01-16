package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DrivePractice extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        double speed, turnspeed, strafespeed;
        boolean override = false;
        double forwardAngle = 0;
        double angle;

        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        DcMotorEx[] motors = { frontLeft, frontRight, backLeft, backRight };

        for(DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            angle = imu.getAngularOrientation().firstAngle - forwardAngle;

            speed = override ? -gamepad1.left_stick_y * 0.7 : -gamepad1.left_stick_y * Math.cos(angle) - gamepad1.left_stick_x * Math.sin(angle);
            turnspeed = gamepad1.right_stick_x * 0.7;
            strafespeed = override ? gamepad1.left_stick_x * 0.7 : -gamepad1.left_stick_y * Math.sin(angle) + gamepad1.left_stick_x * Math.cos(angle);

            frontRight.setPower(speed - turnspeed - strafespeed);
            frontLeft.setPower(speed + turnspeed + strafespeed);
            backRight.setPower(speed - turnspeed + strafespeed);
            backLeft.setPower(speed + turnspeed - strafespeed);

            if (gamepad1.a) {
                override = false;
            }
            if (gamepad1.b) {
                override = true;
            }
            if (gamepad1.y) {
                forwardAngle = imu.getAngularOrientation().firstAngle;
            }

            telemetry.addData("Relative angle", angle);
            telemetry.addData("Forward speed", speed);
            telemetry.addData("Strafe speed", strafespeed);
            telemetry.update();
        }
    }
}
