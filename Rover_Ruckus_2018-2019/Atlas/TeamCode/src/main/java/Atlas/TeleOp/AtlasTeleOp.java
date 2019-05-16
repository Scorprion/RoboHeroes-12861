package Atlas.TeleOp;

import android.database.sqlite.SQLiteDiskIOException;
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
    //Making the slower arm and elbow toggle (driver 2)
    private ElapsedTime openClaw = new ElapsedTime();
    private boolean switchedS = false;
    private boolean usedRecently = false;
    private double controlSpeedE = 1, controlSpeedS = 1;

    //Making the slower robot toggle (driver 1)
    private ElapsedTime rmove = new ElapsedTime();
    private ElapsedTime latching = new ElapsedTime();

    private boolean latchReset = false;
    private boolean robotCSpeed = false; // the boolean for the robot's speed to be able to slow it down
    private boolean robotUsedRecent = false;
    private double robotControlSpeed = 0.7;

    private double upShoulderSpeed = 0, downShoulderSpeed = 0;
    private double turnspeed = 0;
    private double speed = 0;
    private double ElbowSpeed = 0;

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
        upShoulderSpeed = (gamepad2.right_trigger ) * controlSpeedS;
        downShoulderSpeed = (gamepad2.left_trigger ) * controlSpeedS;
        LElbowSpeed = (gamepad2.left_stick_y * 1) * controlSpeedE;
        turnspeed = gamepad1.right_stick_x * robotControlSpeed;
        speed = gamepad1.left_stick_y * robotControlSpeed;



        telemetry.addData("Elbow Speed:", LElbowSpeed);
        telemetry.addData("Up Shoulder Speed:", upShoulderSpeed);
        telemetry.addData("Down Shoulder Speed:", downShoulderSpeed);
        telemetry.addData("Left and Right move power:", speed);
        telemetry.addData("Slowdown used recently:", usedRecently);
        telemetry.addData("Switched Arm Speed:", switchedS);
        telemetry.addData("Robot speed:", speed);
        telemetry.addData("Robot turn speed:", turnspeed);
        telemetry.addData("Robot switched", robotCSpeed);
        telemetry.addData("Robot switched recently:", robotUsedRecent);
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

        //Controlling Shoulder arm speed
        if (gamepad2.a && !usedRecently) {
            if (switchedS) {
                controlSpeedS = 1;
                controlSpeedE = 1;
                switchedS = false;
            } else if (!switchedS) {
                controlSpeedS = 0.7;
                controlSpeedE = 0.7;
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
        //Resetting Latch
        if (gamepad1.y) {
            robot.Sliding.setPosition(0);
            robot.Latching.setPower(-1);
            latchReset = true;
            latching.reset();
        }
        //Turning
        if (gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_x <= -0.1) {
            robot.Left.setPower(-turnspeed);
            robot.Right.setPower(turnspeed);
        }

        //Moving
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= -0.1) {
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);
        }

        //Making the robot stop when it's set to 0
        if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.left_stick_x == 0 ) {
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

        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            Winch.setPower(0);
        }

        if(gamepad1.x && !robotUsedRecent) {
            if (robotCSpeed) {
                robotControlSpeed = 0.7;
                robotCSpeed = false;
            } else if (!robotCSpeed) {
                robotControlSpeed = 0.4;
                robotCSpeed = true;
            }
            robotUsedRecent = true;
            rmove.reset();
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

        } else if (gamepad1.dpad_down || latchReset) {
            Latching.setPower(-1);

        } else
            Latching.setPower(0);

        // Setting the used recently boolean to true after 200
        // milliseconds after the a button was pressed
        if (robot.runtime.milliseconds() > 200) {
            usedRecently = false;
        }

        if(rmove.milliseconds() > 200) {
            robotUsedRecent = false;
        }

        // Setting the LClamp power to 0.5 after the open claw is greater than 250 milliseconds
        if(openClaw.milliseconds() > 200 && openClaw.milliseconds() < 400) {
            robot.LClamp.setPosition(0.5);
        }

        if(latching.milliseconds() > 1200 && latching.milliseconds() < 1400 && latchReset) {
            robot.Latching.setPower(0);
            latchReset = false;
        }
    }
}
