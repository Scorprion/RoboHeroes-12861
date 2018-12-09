package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

import Atlas.Autonomous.MoveMethods.AtlasEncoderDrive;
import Atlas.HardwareAtlas;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends LinearOpMode {
    HardwareAtlas robot = new HardwareAtlas(); //Using the robot hardware

    double moveSpeed = 0.6;
    double turnSpeed = 0.5;
    AtlasEncoderDrive drive = new AtlasEncoderDrive();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //Resetting left and right wheel encoders
        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set mode for the motors to make them run with encoders
        robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        drive.encoderDrive(robot, moveSpeed, 5, 5, 5);
    }
}
