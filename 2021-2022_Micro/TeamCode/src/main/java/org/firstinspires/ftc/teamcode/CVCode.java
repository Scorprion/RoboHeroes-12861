package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;
import java.util.List;

@Autonomous
public class CVCode extends LinearOpMode {
    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

    @Override
    public void runOpMode() throws InterruptedException {
        // Adds live view port for the driver hub
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Opening async. to avoid the thread from waiting for camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                FindColors pipeline = new FindColors();
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED); // this may need to be removed, but I left it in for testing
                camera.startStreaming(1280, 720);
                camera.setPipeline(pipeline);
                List<MatOfPoint> contours = pipeline.getContourList();
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}

class FindColors extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat bitEdges = new Mat();
    List<MatOfPoint> contours;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        // Detects Red
        Core.inRange(hsv, new Scalar(4, 123, 0), new Scalar(11, 255, 255), bitEdges);

        Imgproc.findContours(bitEdges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        return null;
    }

    public List<MatOfPoint> getContourList() {
        return contours;
    }
}