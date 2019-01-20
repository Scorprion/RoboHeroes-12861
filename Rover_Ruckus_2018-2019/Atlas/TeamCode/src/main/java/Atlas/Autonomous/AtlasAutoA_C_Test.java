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

import static Atlas.Autonomous.Init.AggregatedClass.direction.CW;
import static Atlas.Autonomous.Init.AggregatedClass.direction.CCW;


@Autonomous(name = "AtlasAutoA_C_Test", group = "Test")
public class AtlasAutoA_C_Test extends AggregatedClass {
    boolean loop = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Turn on light through the program
        if (robot.ColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.ColorSensor).enableLight(true);
        }


        waitForStart();
        robot.LClamp.setPosition(0);
        movement();
    }

    public void cs() {
        //for measuring the distance
        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Right.setPower(0.15);
        robot.Left.setPower(0.15); //Move at 0.15 speed backwards

        robot.runtime.reset();

        while (opModeIsActive() && !colorFound) {
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            int color = colors.toColor();

            diffred = 0; //colors.red - Color.red(defaultRed);
            diffblue = 0; //colors.blue - Color.blue(defaultBlue);


            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            telemetry.addData("Left Encoder:", robot.Left.getCurrentPosition() + (int)countsPerInch);
            telemetry.addData("Right Encoder", robot.Right.getCurrentPosition() + (int)countsPerInch);
            telemetry.addData("Red:", colors.red);
            telemetry.addData("Calibrated Red:", diffred);
            telemetry.addData("Blue:", colors.blue + diffblue);
            telemetry.addData("Calibrated Blue:", diffblue);
            telemetry.update();

            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold. After that, it drops our team marker (the marker() method)
            //   Detects the gold mineral                                 Detects silver mineral
            /*if ((Color.red(color) > Color.blue(color) + 100 && Color.green(color) > Color.blue(color) + 100)
                    || Math.abs(Color.red(color) - Color.blue(color)) < 20) {

            }*/


            /**
             * Testing with encoders for distance
             */
            if(Color.red(color) >= (85 + diffred) && Color.blue(color) <= (74 + diffblue)) {
                if(robot.Left.getCurrentPosition() + (int)countsPerInch < 1515 && robot.Right.getCurrentPosition() + (int)countsPerInch < 1515) {
                    position1AC(1);
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch <= 3026 && robot.Right.getCurrentPosition() + (int)countsPerInch <= 3026 && robot.Left.getCurrentPosition() + (int)countsPerInch >= 1530 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 1530) {
                    position2AC(1);
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch <= 4570 && robot.Right.getCurrentPosition() + (int)countsPerInch <= 4570 && robot.Left.getCurrentPosition() + (int)countsPerInch >= 3030 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 3030) {
                    position3AC(1);
                } else {
                    stopMotors();
                }
            }



            /*if ((Color.red(color) >= 80 && Color.blue(color) <= 79) || (Color.red(color) >= 70)) {
                counter++;
                sleep(100);
            }

            if (Color.red(color) >= 80 && Color.blue(color) <= 79 && counter == 1) {
                position1();
                marker();
            }
            else if (Color.red(color) >= 80 && Color.blue(color) <= 79 && counter == 2) {
                position2();
                marker();
            }
            else if (Color.red(color) >= 80 && Color.blue(color) <= 79 && counter == 3) {
                position3();
                marker();
            }
            else if (Color.red(color) >= 80 && counter == 3) {
                position3();
                marker();
            }
            else if(robot.runtime.seconds() > 5) {
                stopMotors();
            }*/
        }
    }

    public void movement() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        /*robot.Latching.setPower(0.8);
        robot.Winch.setPower(-1);
        sleep(2000);
        robot.Sliding.setPosition(1);
        sleep(500);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(1000);*/
        //calibrateCS();
        encoderDrives(0.4, 18, 18);
        sleep(500);
        position1AC();
        encoderDrives(0.4, -5, -5);
        sleep(500);
        PIDCCW(0.4, 0.4, 0, 40);
        sleep(1000);
        encoderDrives(0.4, 12, 12);
        sleep(500);
        encoderDrives(0.4, -11, -11);
        sleep(500);
        PIDCW(0.3, 0.4, 0, 320);
        sleep(500);
        encoderDrives(0.4, 12, 12);
    }
}

