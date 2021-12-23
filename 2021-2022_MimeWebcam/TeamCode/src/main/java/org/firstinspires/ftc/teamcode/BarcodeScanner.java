package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.Point;

@Autonomous
public class BarcodeScanner extends LinearOpMode {
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        // Adds live view port for the driver hub
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED); // this may need to be removed, but I left it in for testing
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        ColorScanner pipeline = new ColorScanner();
        camera.setPipeline(pipeline);


        // Opening async. to avoid the thread from waiting for camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Position", pipeline.getLocation());
            telemetry.update();
        }
    }
}