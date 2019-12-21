package org.firstinspires.ftc.teamcode.LiftBot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name= "TeleOp LiftBot", group= "Pushbot")
public class LiftBotTeleOp extends OpMode {

    CRServo Left;
    CRServo Right;
    CRServo Grip1;
    CRServo Grip2;
    CRServo TurnGrip;

    public void init() {
        Left = hardwareMap.get(CRServo.class, "Left");
        Right = hardwareMap.get(CRServo.class, "Right");
        Grip1 = hardwareMap.get(CRServo.class, "Grip1");
        Grip2 = hardwareMap.get(CRServo.class, "Grip2");
        Grip2.setDirection(REVERSE);
        TurnGrip = hardwareMap.get(CRServo.class, "TurnGrip");
    }

    public void loop() {

        if (gamepad1.left_stick_y >= 0 || gamepad1.left_stick_y <= 0) {
            double upDown = gamepad1.left_stick_y;

            if (gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_y >= 0.1) {
                Left.setPower(upDown);
                Right.setPower(-upDown);
            } else {
                Left.setPower(0);
                Right.setPower(0);
            }

            if (gamepad1.a) {
                Grip1.setPower(1);
                Grip2.setPower(1);
            } else {
                Grip1.setPower(0);
                Grip1.setPower(0);
            }
        }
    }
}