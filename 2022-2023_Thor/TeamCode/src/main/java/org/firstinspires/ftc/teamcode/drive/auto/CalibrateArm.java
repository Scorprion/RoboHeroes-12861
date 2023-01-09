/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.ThorDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class CalibrateArm extends LinearOpMode
{
    ThorDrive drive;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        drive = new ThorDrive(hardwareMap);
        drive.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Calibration Ready.");
        telemetry.update();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 5000) {
            drive.pivot.setPower(-0.15);
            telemetry.addLine("Calibrating pivot...");
            telemetry.addData("Pivot counts", drive.pivot.getCurrentPosition());
            telemetry.update();
        }
        sleep(250);
        drive.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.pivot.setPower(0);
        sleep(250);
        drive.pivot.setTargetPosition(150);
        drive.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.pivot.setPower(0.15);

        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 7000) {
            drive.lift.setPower(0.6);
            telemetry.addLine("Calibrating lift...");
            telemetry.addData("Lift counts", drive.lift.getCurrentPosition());
            telemetry.update();
        }

        drive.lift.setPower(0);
        drive.pivot.setPower(0);
        sleep(500);
        drive.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Calibrated. Resetting robot...");
        telemetry.update();

        drive.lift.setTargetPosition(-4000);
        drive.pivot.setTargetPosition(0);
        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.lift.setPower(0.6);

        sleep(2500);
        drive.pivot.setPower(0.15);
        sleep(1000);

        drive.lift.setPower(0);
        drive.pivot.setPower(0);

        telemetry.addLine("Finished calibration.");
        telemetry.update();
        sleep(30000);
    }
}