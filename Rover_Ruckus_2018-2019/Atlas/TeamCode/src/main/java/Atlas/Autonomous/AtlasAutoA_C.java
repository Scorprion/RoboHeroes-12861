package Atlas.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Atlas.Autonomous.Init.AggregatedClass;
import Atlas.Autonomous.Init.HardwareAtlas;


@Autonomous(name = "AtlasAutoA_C", group = "Auto")
public class AtlasAutoA_C extends AggregatedClass {

    public boolean colorFound = false;
    int counter = 0;

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


        robot.Left.setPower(0.2); //Move at 0.2 speed backwards
        robot.Right.setPower(0.2);
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

            telemetry.addData("Counter:", counter);
            telemetry.addData("Left Encoder:", robot.Left.getCurrentPosition() + (int)countsPerInch);
            telemetry.addData("Right Encoder", robot.Right.getCurrentPosition() + (int)countsPerInch);
            telemetry.update();

            // Detects a change in the color and then stops robot after the red or blue values
            // reach a certain threshold. After that, it drops our team marker (the marker() method)
            //   Detects the gold mineral                                 Detects silver mineral
            /*if ((Color.red(color) > Color.blue(color) + 100 && Color.green(color) > Color.blue(color) + 100)
                    || Math.abs(Color.red(color) - Color.blue(color)) < 20) {

            }*/


            /*if(Color.red(color) > Color.blue(color) + 100 && Color.green(color) > Color.blue(color) + 100 && counter == 1) {
                counter1();
                marker();
            } else if(Color.red(color) > Color.blue(color) + 100 && Color.green(color) > Color.blue(color) + 100 && counter == 2) {
                counter2();
                marker();
            } else if(Color.red(color) > Color.blue(color) + 100 && Color.green(color) > Color.blue(color) + 100 && counter == 3) {
                counter3();
                marker();
            } else if(checkSilver(color)) {
                counter3();
                marker();
            } else if(robot.runtime.seconds() > 5) {
                stopMotors();
                telemetry.addLine("The sampling failed rip");
            }*/


            /**
             * Testing with encoders for distance
             */

            if(Color.red(color) >= 80 && Color.blue(color) <= 79) {
                if(robot.Left.getCurrentPosition() + (int)countsPerInch < 12 && robot.Right.getCurrentPosition() + (int)countsPerInch < 12) {
                    position1();
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch >= 12 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 12) {
                    position2();
                } else if(robot.Left.getCurrentPosition() + (int)countsPerInch >= 24 && robot.Right.getCurrentPosition() + (int)countsPerInch >= 24) {
                    position3();
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

    public void marker() {
        sleep(1000);
        robot.Left.setPower(0.8); //Move toward the blue line at 0.2 speed
        robot.Right.setPower(0.8);
        sleep(250); //added second pause also for debugging
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

            if (Color.blue(color) >= 125 || Color.red(color) >= 140) {
                sleep(500);
                robot.Marker.setPosition(0);
            }
        }
    }

    public void movement() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        /*robot.Latching.setPower(0.75);
        robot.Winch.setPower(-1);
        sleep(3250);
        robot.Latching.setPower(-1);
        sleep(500);
        robot.Latching.setPower(0);
        robot.Winch.setPower(0);
        stopMotors();
        sleep(2000);*/
        encoderDrives(1, 4, 4);
        sleep(500);
        proportional(0.5, 60, 3, 3);
        sleep(500);
        encoderDrives(0.4, 30, 30);
        sleep(500);
        proportional(0.5, 95, 3, 4);
        sleep(500);
        cs();
    }
}





   /* public void forward(double speed, double seconds) {
        double time = seconds * 1000;/

        //Move at the speed "speed" and pause for "seconds" amount of time before stopping
        robot.Left.setPower(speed);
        robot.Right.setPower(speed);
        sleep((long)time);
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    public void turn(double speed, double seconds) {
        double time = seconds * 1000;

        //turn at speed "speed" and pause for "seconds" amount of time before stopping
        robot.Left.setPower(-speed);
        robot.Right.setPower(speed);
        sleep((long)time);
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }

    public void stopMotion() {
        robot.Left.setPower(0);
        robot.Right.setPower(0);
    }
}
*/
