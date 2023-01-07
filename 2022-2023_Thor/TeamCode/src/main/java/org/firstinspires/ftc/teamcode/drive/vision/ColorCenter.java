package org.firstinspires.ftc.teamcode.drive.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.ThorDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class ColorCenter extends LinearOpMode {

    // PID Coeffs
    public static double P = 0.0;
    // public static double I = 0.0;
    // public static double D = 0.0;

    double powerUpdate = 0;
    double correction = 0;
    double error = 0;

    double mapRange(double x, double oldMin, double oldMax, double newMin, double newMax) {
        double range = oldMax - oldMin;
        double ratio = x / range;
        double newRange = newMax - newMin;
        return newRange * ratio;
    }

    double clamp(double x, double min, double max) {
        if (x > max) {
            return max;
        } else if (x < min) {
            return min;
        }
        return x;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Change to ftc dashboard telemetry
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot and camera
        ThorDrive drive = new ThorDrive(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(drive.camera, 0);

        // Initialize pipeline
        LargestContourScanner pipeline = new LargestContourScanner(telemetry);
        drive.camera.setPipeline(pipeline);

        // Center should be around 640, 360 (x, y)
        // Opening async. to avoid the thread from waiting for camera
        drive.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                drive.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Initialized Camera");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            error = pipeline.getCenterError();

            correction = P * error;
            powerUpdate = clamp(correction, -1, 1);
            // powerUpdate = mapRange(correction, -640.0, 640.0, -1.0, 1.0);

            // more experimental
            // correction = mapRange(error, -640.0, 640.0, -1.0, 1.0);
            // powerUpdate = Math.pow(correction, P);

            drive.backLeft.setPower(-powerUpdate);
            drive.frontLeft.setPower(-powerUpdate);
            drive.backRight.setPower(powerUpdate);
            drive.frontRight.setPower(powerUpdate);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }
}