package org.firstinspires.ftc.teamcode.Hermes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HardwareHermes;

@TeleOp(name = "HermesEncoderTest", group = "Calibration")
public class HermesEncoderTest extends LinearOpMode {

    HardwareHermes robot = new HardwareHermes();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double initial_frontright = robot.FrontRight.getCurrentPosition();
        double initial_backright = robot.BackRight.getCurrentPosition();
        double initial_frontleft = robot.FrontLeft.getCurrentPosition();
        double initial_backleft = robot.BackLeft.getCurrentPosition();

        double right_avg, left_avg, right_difference, left_difference, countsperinch = 73.5365;

        while(opModeIsActive()) {
            right_avg = robot.FrontRight.getCurrentPosition() + robot.BackRight.getCurrentPosition() / 2;
            left_avg = robot.FrontLeft.getCurrentPosition() + robot.BackLeft.getCurrentPosition() / 2;
            telemetry.addData("Right1: ", right_avg);
            telemetry.addData("Left1: ", left_avg);

            telemetry.addData("Right2: ", (robot.FrontRight.getCurrentPosition() + robot.BackRight.getCurrentPosition()) / 2);
            telemetry.addData("Left2: ", (robot.FrontLeft.getCurrentPosition() + robot.BackLeft.getCurrentPosition()) / 2);

            right_difference = Math.abs(right_avg);
            left_difference = Math.abs(left_avg);
            telemetry.addData("Right Difference: ", right_difference);
            telemetry.addData("Left Difference: ", left_difference);

            telemetry.addData("Average: ", Math.abs(right_difference + left_difference) / 2);

            telemetry.addLine()
                    .addData("Right Inches: ", right_avg / countsperinch)
                    .addData("Left Inches: ", left_avg / countsperinch);
            telemetry.update();
        }
    }
}