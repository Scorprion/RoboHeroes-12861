package Atlas.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

import Atlas.Autonomous.Init.HardwareAtlas;

@TeleOp(name= "AtlasTeleOp", group= "Pushbot")
public class AtlasTeleOp extends OpMode {
    double ShoulderSpeed = 0;
    double ElbowSpeed = 0;
    double turnspeed = 0;
    double speed = 0;

    double LElbowSpeed = 0;

    DcMotor Winch;
    CRServo Latching;


    //Set the speed of the motors when the Left or Right sticks are not idle

    HardwareAtlas robot = new HardwareAtlas();

    public void init() {
        robot.init(hardwareMap);
        Winch = hardwareMap.dcMotor.get("Winch");
        Latching = hardwareMap.crservo.get("Latching");
        //DriveL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        ShoulderSpeed = gamepad2.right_trigger;

        LElbowSpeed = gamepad2.left_stick_y * 0.8;
        turnspeed = gamepad1.right_stick_x;
        speed = gamepad1.left_stick_y;
        telemetry.addData("The speed for both motors", speed);
        telemetry.addData("The speed for both motors in turning", turnspeed);

        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 2         |
        |                           |
        |                           |
        -----------------------------
         */
        //The Shoulders
        if (gamepad2.right_trigger >= 0.1) {
            robot.LShoulder.setPower(ShoulderSpeed);
        }

        if (gamepad2.left_trigger >= 0.1) {
            robot.LShoulder.setPower(-ShoulderSpeed);

        }

        //The LElbow
        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            robot.LElbow.setPower(LElbowSpeed);
        } else {
            ElbowSpeed = 0;
        }

        if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {

        }

        //The LClamp
        if (gamepad2.right_bumper) {
            robot.LClamp.setPosition(1);
            try {
                //Wait 250 milliseconds before stopping the movement of the clamp
                TimeUnit.MILLISECONDS.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.LClamp.setPosition(0.5);
        }
        if (gamepad2.left_bumper) {
            robot.LClamp.setPosition(0);
        }



        /*
        -----------------------------
        |                           |
        |                           |
        |         Gamepad 1         |
        |                           |
        |                           |
        -----------------------------
         */
        //Turning
        if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1) {
            robot.Left.setPower(-turnspeed);
            robot.Right.setPower(turnspeed);
        } else {
            robot.Left.setPower(0);
            robot.Right.setPower(0);
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);
        } else {
            robot.Left.setPower(0);
            robot.Right.setPower(0);
        }

        //Tilting and resetting back our marker servo platform
        if (gamepad1.a) {
            robot.Marker.setPosition(1.0);
        }
        if (gamepad1.b) {
            robot.Marker.setPosition(0);
        }
        //The winch
        if (gamepad1.left_trigger > 0.1) {
            Winch.setPower(gamepad1.left_trigger);
        }

        if (gamepad1.right_trigger > 0.1) {
            Winch.setPower(-gamepad1.right_trigger);
        }

        if (gamepad1.y) {
            Winch.setPower(0);
        }

        //The latching
        if (gamepad1.dpad_up) {
            Latching.setPower(1);

        } else if (gamepad1.dpad_down) {
            Latching.setPower(-1);

        } else
            Latching.setPower(0);
        }



}
