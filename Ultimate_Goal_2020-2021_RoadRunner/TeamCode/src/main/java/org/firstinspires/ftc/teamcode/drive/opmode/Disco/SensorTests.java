package org.firstinspires.ftc.teamcode.drive.opmode.Disco;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants.RingPosition;

import java.util.Arrays;
import java.util.List;

@TeleOp(group="Disco")
public class SensorTests extends OpMode {
    DcMotorEx leftFront, leftRear, rightRear, rightFront;
    DistanceSensor ringSensor;
    List<DcMotorEx> motors;

    RingPosition conclusion;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontRight");

        ringSensor = hardwareMap.get(DistanceSensor.class, "RingSensor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // 0.75 inches (rings)     4.9 inches off ground
    }

    @Override
    public void loop() {
        double distance = ringSensor.getDistance(DistanceUnit.INCH);
        if(distance >= 5) {
            conclusion = RingPosition.NONE;
        } else if (distance >= 3) {
            conclusion = RingPosition.SINGLE;
        } else {
            conclusion = RingPosition.FOUR;
        }


        telemetry.addData("Front left", leftFront.getCurrentPosition());
        telemetry.addData("Front right", rightFront.getCurrentPosition());
        telemetry.addData("Distance:", distance);
        telemetry.addData("Conclusion", conclusion);
        telemetry.update();
    }
}
