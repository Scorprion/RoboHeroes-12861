package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="QuickTest", group = "PushBot")
public class QuickTest extends LinearOpMode {

    public DcMotor frontRight;
    public DcMotor frontLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setPower(0.1);
        frontLeft.setPower(0.1);
        sleep(1000);
        frontRight.setPower(-0.1);
        frontLeft.setPower(-0.1);
        sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);

    }
}
