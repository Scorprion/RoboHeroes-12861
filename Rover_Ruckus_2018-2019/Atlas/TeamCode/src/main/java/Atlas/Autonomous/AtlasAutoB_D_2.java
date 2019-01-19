package Atlas.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;

import static Atlas.Autonomous.Init.AggregatedClass.direction.CCW;


@Autonomous(name = "AtlasAutoB_D_2", group = "Auto")
public class AtlasAutoB_D_2 extends AggregatedClass {

    HardwareAtlas robot = new HardwareAtlas();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Turn on light through the program
        if (robot.ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.ColorSensor).enableLight(true);
        }

        waitForStart();
        movement();
    }
    public void cs() {
        //for measuring the distance
        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Right.setPower(0.2);
        robot.Left.setPower(0.2); //Move at 0.2 speed backwards

        robot.runtime.reset();
        while (opModeIsActive() && !colorFound) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addData("Left Encoder:", robot.Left.getCurrentPosition() + (int) countsPerInch);
            telemetry.addData("Right Encoder", robot.Right.getCurrentPosition() + (int) countsPerInch);
            telemetry.update();

            if(Color.red(color) >= 85 && Color.blue(color) <= 74) {
                if(robot.Left.getCurrentPosition() + (int)countsPerInch < 1515 && robot.Right.getCurrentPosition() + (int)countsPerInch < 1515) {
                    position1BD(2);
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch <= 3026 && robot.Right.getCurrentPosition() + (int)countsPerInch <= 3026 && robot.Left.getCurrentPosition() + (int)countsPerInch >= 1530 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 1530) {
                    position2BD(2);
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch <= 4570 && robot.Right.getCurrentPosition() + (int)countsPerInch <= 4570 && robot.Left.getCurrentPosition() + (int)countsPerInch >= 3030 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 3030) {
                    position3BD(2);
                } else {
                    stopMotors();
                }
            }
        }

    }


    public void movement() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        /*robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        sleep(3250);
        encoderDrives(0.5, 1, -1);
        sleep(250);
        robot.Latching.setPower(-1);
        sleep(500);
        encoderDrives(0.5, -1, 1);
        sleep(250);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);*/
        encoderDrives(1, 5, 5);
        sleep(1000);
        //proportional(0.5, 60, 3);
        encoderDrives(0.5, -8, 8);
        sleep(250);
        encoderDrives(0.4, 29, 29);
        sleep(250);
        proportional(CCW,0.4, 93, 3,4);
        cs();
    }
}
