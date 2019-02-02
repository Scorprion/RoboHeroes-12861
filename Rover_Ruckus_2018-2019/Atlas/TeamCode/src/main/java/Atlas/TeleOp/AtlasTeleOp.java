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

@TeleOp(name= "AtlasTeleOp", group= "Pushbot")
public class AtlasTeleOp extends OpMode {
    private ElapsedTime openClaw = new ElapsedTime();
    private boolean switchedS = false;
    private boolean usedRecently = false;
    private double controlSpeed = 1;
    private double upShoulderSpeed = 0, downShoulderSpeed = 0;
    private double ElbowSpeed = 0;
    private double turnspeed = 0;
    private double speed = 0;

    private double LElbowSpeed = 0;

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

        upShoulderSpeed = (gamepad2.right_trigger * 0.7) * controlSpeed;
        downShoulderSpeed = (gamepad2.left_trigger * 0.7) * controlSpeed;
        LElbowSpeed = gamepad2.left_stick_y * controlSpeed;
        turnspeed = gamepad1.right_stick_x * 0.5;
        speed = gamepad1.left_stick_y * 0.7;
        telemetry.addData("Elbow Speed:", LElbowSpeed);
        telemetry.addData("Up Shoulder Speed:", upShoulderSpeed);
        telemetry.addData("Down Shoulder Speed:", downShoulderSpeed);
        telemetry.addData("Left and Right move power:", speed);
        telemetry.addData("Slowdown used recently:", usedRecently);
        telemetry.addData("Switched Speed:", switchedS);
        telemetry.update();

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
            robot.LShoulder.setPower(upShoulderSpeed);
        }

        if (gamepad2.left_trigger >= 0.1) {
            robot.LShoulder.setPower(-downShoulderSpeed);

        }

        //The LElbow
        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            robot.LElbow.setPower(LElbowSpeed);
        } else {
            ElbowSpeed = 0;
        }

        //Controlling arm speed
        if (gamepad2.a && !usedRecently) {
            if (switchedS) {
                controlSpeed = 1;
                switchedS = false;
            } else if (!switchedS) {
                controlSpeed = 0.5;
                switchedS = true;
            }
            usedRecently = true;
            robot.runtime.reset();
        }

        //The LClamp
        if (gamepad2.right_bumper) {
            robot.LClamp.setPosition(1);
            openClaw.reset();
            //Wait 250 milliseconds before stopping the movement of the clamp

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
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);
        }

        //Making the robot stop when it's set to 0
        if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
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
        if (gamepad1.dpad_left) {
            robot.Sliding.setPosition(1);
        }

        if (gamepad1.dpad_right) {
            robot.Sliding.setPosition(0);
        }

        if (gamepad1.dpad_up) {
            Latching.setPower(1);

        } else if (gamepad1.dpad_down) {
            Latching.setPower(-1);

        } else
            Latching.setPower(0);

        // Setting the used recently boolean to true after 200
        // milliseconds after the a button was pressed
        if (robot.runtime.milliseconds() > 200) {
            usedRecently = false;
        }

        // Setting the LClamp power to 0.5 after the open claw is greater than 250 milliseconds
        if(openClaw.milliseconds() > 200 && openClaw.milliseconds() < 400) {
            robot.LClamp.setPosition(0.5);
        }
    }
}
