package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends AggregatedClass {
    HardwareAtlas robot = new HardwareAtlas(); //Using the robot hardware

    //Set the move and turn speed for the robot
    double moveSpeed = 0.2;
    double turnSpeed = 0.5;

    //Use the AtlasEncoderDrive class to control the encoders

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

        //Since this class extends from the AggregatedClass, it can just call to the encoderDrive
        //method initialized there
    }
}
