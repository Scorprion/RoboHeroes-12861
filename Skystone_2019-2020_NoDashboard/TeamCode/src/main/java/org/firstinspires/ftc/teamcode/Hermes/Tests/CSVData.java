package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.PID;

import java.io.IOException;

@TeleOp(name = "CSVData", group = "Hermes")
public class CSVData extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    double speed = 0, turnspeed = 0, strafespeed = 0;
    double average = 0;
    double last_time = 0;
    double last_angle = 0;
    double total_angle = 0;
    double op_time = 0;
    double current_angle = 0;

    DataLogger d;
    {
        try {
            d = new DataLogger("test.csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    DcMotor BackLeft, BackRight, FrontLeft, FrontRight;
    BNO055IMU imu;

    public void init() {
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        try {
            d.addHeaderLine("Time," + "InputBL," + "InputBR," + "InputFL," + "InputFR," +
                    "Angle," + "Ang_Velocity," + "X_Pos" + "Y_Pos"
                    + "EncoderBL," + "EncoderBR," + "EncoderFL," + "EncoderFR");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        op_time = timer.milliseconds() / 1000;

        turnspeed = -gamepad1.right_stick_x * 0.7;
        strafespeed = gamepad1.left_stick_x * 0.7;
        speed = gamepad1.left_stick_y * -0.7;

        BackLeft.setPower(speed - strafespeed + turnspeed);
        BackRight.setPower(speed + strafespeed - turnspeed);
        FrontLeft.setPower(speed + strafespeed + turnspeed);
        FrontRight.setPower(speed - strafespeed - turnspeed);

        average = Math.abs(FrontRight.getCurrentPosition() +
                BackRight.getCurrentPosition()) / 2;
        current_angle = likeallelse(imu.getAngularOrientation().firstAngle);

        total_angle += current_angle - last_angle;

        telemetry.addData("Time", op_time);
        telemetry.addData("Real time", time);
        telemetry.addData("Input", speed);
        telemetry.addData("Angle", total_angle);
        telemetry.addData("Angular Velocity", imu.getAngularVelocity().xRotationRate);
        telemetry.addData("Encoder", average);
        telemetry.update();

        try {
            d.addDataLine(op_time + "," +
                    (speed - strafespeed + turnspeed) + "," +
                    (speed + strafespeed - turnspeed) + "," +
                    (speed + strafespeed + turnspeed) + "," +
                    (speed - strafespeed - turnspeed) + "," +
                    current_angle + "," +
                    imu.getAngularVelocity().xRotationRate + "," +
                    imu.getPosition().x + "," +
                    imu.getPosition().y + "," +
                    BackLeft.getCurrentPosition() + "," +
                    BackRight.getCurrentPosition() + "," +
                    FrontLeft.getCurrentPosition() + "," +
                    FrontRight.getCurrentPosition());
        } catch (IOException e) {
            e.printStackTrace();
        }

        last_time = op_time;
        last_angle = current_angle;
    }

    public void stop() {
        d.close();
    }

    private double likeallelse(double angle) {
        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;

        return angle;
    }
}