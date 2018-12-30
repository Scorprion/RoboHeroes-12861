/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package TestBot;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.sql.Driver;
import java.util.Locale;

import TestBot.Init.HardwareTestBot;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class ACAuto extends AggregatedTestBot {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public boolean colorFound = false;

    final double countsPerRot = 1120; // The counts per rotation
    final double driveGearReduction = 1; // The drive gear reduction value for the robot
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    final double countsPerInch = (countsPerRot * driveGearReduction) / (wheelDiamInch * 3.1415);
    static final double driveSpeed = 0.6;
    final double HEADING_THRESHOLD = 1;
    final double P_TURN_COEFF = 0.1; // Larger is more responsive, but also less stable
    final double P_DRIVE_COEFF = 0.15; // Larger is more responsive, but also less stable
    public final double turnSpeed = 0.5;

    double speed = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.Left.getCurrentPosition(),
                robot.Right.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

           movement();



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void cs() {
        sleep(1000);
        encoderDrive(-0.4, 13, 13, 5); //Drive backwards at 0.4 speed for 13 inches
        sleep(500);
        while(!colorFound && opModeIsActive()) {
            float[] hsvValues = new float[3];
            final float values[] = hsvValues;
            robot.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            NormalizedRGBA colors = robot.ColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();

            if(Color.red(color) >= 90 && Color.green(color) >= 80) {
                sleep(1000);
                proportional(turnSpeed, 0, 3);
                sleep(1000);
                encoderDrive(0.4, 9,9, 5);
            }
        }
    }

    public void encoderDrive(double speed, double linches, double rinches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (rinches * countsPerInch);
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(-Math.abs(speed));
            robot.Right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.Left.isBusy() && robot.Right.isBusy())&& (runtime.seconds() < timeoutS)) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d");
                robot.Left.getCurrentPosition();
                robot.Right.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            robot.Left.setPower(0);
            robot.Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*public enum direction {
        CW, CCW
    }*/

    public void movement() {
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(driveSpeed, 12, 12);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(1000);
        encoderDrive(driveSpeed, 4, 4, 5);
        proportional(turnSpeed, 60, 4); //Turn ccw to 45 degrees with 3 corrections
    }
}
