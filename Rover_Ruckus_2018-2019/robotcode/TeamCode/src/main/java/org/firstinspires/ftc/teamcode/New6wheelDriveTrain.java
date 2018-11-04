package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name= "ExampleBot2019-2", group= "Pushbot")
public class New6wheelDriveTrain extends OpMode{

        public DcMotor Right;
        public DcMotor Left;
        public DcMotor RightCore;
        public DcMotor LeftCore;
        double speed = 0;

        public void init() {
            Right = hardwareMap.dcMotor.get("backRight");
            Left = hardwareMap.dcMotor.get("backLeft");


            Left.setDirection(DcMotor.Direction.REVERSE);
            LeftCore.setDirection(DcMotor.Direction.REVERSE);
        }

        @Override
        public void loop() {

            speed = gamepad1.left_stick_y * 1;


            if (gamepad1.right_stick_x >= 0.1) {

                Left.setPower(-1);
                Right.setPower (1);
            } else if (gamepad1.right_stick_x >= 0.9) {

                Left.setPower(-1);
                Right.setPower(1);
            }
            if (gamepad1.right_stick_x <= -0.1) {

                Left.setPower(1);
                Right.setPower(-1);
            } else if (gamepad1.right_stick_x <= -0.9) {

                Left.setPower(1);
                Right.setPower(-1);
            }
            if (gamepad1.a) {

                LeftCore.setPower(1);
                RightCore.setPower(-1);
            }




            Right.setPower(speed);
            Left.setPower(speed);



        }
}
