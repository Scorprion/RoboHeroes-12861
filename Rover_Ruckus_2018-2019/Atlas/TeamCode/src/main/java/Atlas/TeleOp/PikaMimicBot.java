package Atlas.TeleOp;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import Atlas.Autonomous.Init.HardwareAtlas;

@TeleOp(name= "PikaMimic", group= "Pushbot")
public class PikaMimicBot extends OpMode {
    private double LElbowSpeed = 0;

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;


    //Set the speed of the motors when the Left or Right sticks are not idle

    HardwareAtlas robot = new HardwareAtlas();

    public void init() {

        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        //DriveL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
       double speed = gamepad1.left_stick_y;
       double turn = gamepad1.left_stick_x;

       if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
           FrontLeft.setPower(speed);
           FrontRight.setPower(speed);
           BackLeft.setPower(speed);
           BackRight.setPower(speed);
       }else if (gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1){
           FrontLeft.setPower(turn);
           FrontRight.setPower(-turn);
           BackLeft.setPower(turn);
           BackRight.setPower(-turn);
       }else{
           FrontLeft.setPower(0);
           FrontRight.setPower(0);
           BackLeft.setPower(0);
           BackRight.setPower(0);
       }
    }
}
