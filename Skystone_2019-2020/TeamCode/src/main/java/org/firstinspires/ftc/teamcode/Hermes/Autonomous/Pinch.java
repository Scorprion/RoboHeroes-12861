package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class Pinch extends LinearOpMode {

    DcMotor FrontRight, FrontLeft, BackRight, BackLeft;
    Servo Clamp;

    static final double COUNTS_PER_MOTOR = 560;
    static final double WHEEL_DIAMETER_INCH = 560/6.3*3.1415;

    @Override
    public void runOpMode() throws InterruptedException {

        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        Clamp = hardwareMap.get(Servo.class, "Clamp");

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


    }

    public void encoderRun(double speed, double FrontRightInches, double FrontLeftInches, double BackRightInches, double BackLeftInches) {
        int newFrontRightTarget;
        int newFrontLeftTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        if (opModeIsActive()) {

        }
    }



}
