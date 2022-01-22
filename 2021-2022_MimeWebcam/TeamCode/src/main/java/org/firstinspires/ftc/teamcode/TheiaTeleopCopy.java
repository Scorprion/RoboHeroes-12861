package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="Theia")
public class TheiaTeleopCopy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            driveTrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, -gamepad1.right_stick_x * 0.7));
            driveTrain.update();

            driveTrain.outtake.setPower(gamepad2.right_stick_y);

            if(gamepad1.a) {
                driveTrain.spintake.setPower(1.0);
            } else if(gamepad1.b) {
                driveTrain.spintake.setPower(-1.0);
            } else {
                driveTrain.spintake.setPower(0.0);
            }

            driveTrain.intakearm.setPower(0.7 * (gamepad1.right_trigger - gamepad1.left_trigger));

            if(gamepad2.a) {
                driveTrain.sorter.setPosition(0.0);
            } else if(gamepad2.b) {
                driveTrain.sorter.setPosition(0.5);
            } else {
                driveTrain.sorter.setPosition(1.0);
            }

            if(gamepad2.x) {
                driveTrain.release.setPower(1.0);
            } else if (gamepad2.y) {
                driveTrain.release.setPower(-1.0);
            } else {
                driveTrain.release.setPower(0.0);
            }

        }
    }
}
