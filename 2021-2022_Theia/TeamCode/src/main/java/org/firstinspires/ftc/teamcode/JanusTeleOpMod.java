package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.JanusDrive;

@TeleOp(group="Janus")
public class JanusTeleOpMod extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
      JanusDrive robot = new JanusDrive(hardwareMap);
      waitForStart();
      if(isStopRequested()) return;

      while(opModeIsActive()) {
         robot.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * 0.8, -gamepad1.left_stick_x * 0.8, gamepad1.right_stick_x * 0.8));
         robot.update();
      }
   }
}
