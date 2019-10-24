package org.firstinspires.ftc.teamcode.Autonomous.SensorTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "EncoderTest", group = "Calibration")
public class EncoderTest extends LinearOpMode {

    DcMotor Right;
    DcMotor Left;

    @Override
    public void runOpMode() throws InterruptedException {
        Right = hardwareMap.get(DcMotor.class, "Right");
        Left = hardwareMap.get(DcMotor.class, "Left");

        Right.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        double initial_right = Right.getCurrentPosition();
        double initial_left = Left.getCurrentPosition();

        while(opModeIsActive()) {
            telemetry.addData("Right1: ", initial_right);
            telemetry.addData("Left1: ", initial_left);

            telemetry.addData("Right2: ", Right.getCurrentPosition());
            telemetry.addData("Left2: ", Left.getCurrentPosition());

            telemetry.addData("Right Difference: ", Math.abs(initial_right - Right.getCurrentPosition()));
            telemetry.addData("Left Difference: ", Math.abs(initial_left - Left.getCurrentPosition()));

            telemetry.addData("Average: ", (Math.abs((initial_left - Left.getCurrentPosition()) + (initial_right - Right.getCurrentPosition()))) / 2);
            telemetry.update();
        }
    }
}