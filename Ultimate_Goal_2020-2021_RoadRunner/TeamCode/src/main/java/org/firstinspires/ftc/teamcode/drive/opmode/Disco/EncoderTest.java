package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;

@TeleOp(group="Disco")
public class EncoderTest extends OpMode {
    DcMotorEx leftFront, leftRear, rightRear, rightFront;
    List<DcMotorEx> motors;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    @Override
    public void loop() {
        telemetry.addData("Front left", leftFront.getCurrentPosition());
        telemetry.addData("Front right", rightFront.getCurrentPosition());
        telemetry.update();
    }
}
