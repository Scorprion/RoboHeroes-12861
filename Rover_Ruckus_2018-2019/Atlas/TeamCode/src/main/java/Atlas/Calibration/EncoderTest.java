package Atlas.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import STC_2019.Left;
import STC_2019.Right;

@TeleOp(name = "EncoderTest", group = "Calibration")
public class EncoderTest extends LinearOpMode {

    DcMotor RightMotor;
    DcMotor LeftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        double initial_right = RightMotor.getCurrentPosition();
        double initial_left = LeftMotor.getCurrentPosition();

        while(opModeIsActive()) {
            telemetry.addData("Right1: ", initial_right);
            telemetry.addData("Left1: ", initial_left);

            telemetry.addData("Right2: ", RightMotor.getCurrentPosition());
            telemetry.addData("Left2: ", LeftMotor.getCurrentPosition());

            telemetry.addData("Right Difference: ", initial_right - RightMotor.getCurrentPosition());
            telemetry.addData("Left Difference: ", initial_left - LeftMotor.getCurrentPosition());

            telemetry.addData("Average: ", ((initial_left - LeftMotor.getCurrentPosition()) + (initial_right - RightMotor.getCurrentPosition())) / 2);
            telemetry.update();
        }
    }
}