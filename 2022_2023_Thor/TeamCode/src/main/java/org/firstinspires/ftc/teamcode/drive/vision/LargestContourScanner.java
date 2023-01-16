package org.firstinspires.ftc.teamcode.drive.vision;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class LargestContourScanner extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat blueEdges = new Mat();
    Mat yellowEdges = new Mat();
    Mat redEdges = new Mat();
    Mat output = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();

    Telemetry telemetry;

    // HSV value for blue
    Scalar blueLower = new Scalar(100, 75, 0);
    Scalar blueUpper = new Scalar(120, 255, 255);

    // HSV for yellow
    Scalar yellowLower = new Scalar(20, 120, 0);
    Scalar yellowUpper = new Scalar(30, 255, 255);

    // HSV for red, yellow, and blue (subtract blue and yellow to get red)
    Scalar rybLower = new Scalar(0, 90, 0);
    Scalar rybUpper = new Scalar(255, 255, 255);

    int x = 640, y = 360;


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Mat processFrame(Mat input) {
        // Resetting all of the variables since the last call to this method
        output.setTo(new Scalar(0, 0, 0));
        contours.clear();

        // Converts image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Detects alliance color
        Core.inRange(hsv, blueLower, blueUpper, blueEdges);

        Imgproc.findContours(blueEdges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create output matrix
        Core.bitwise_and(input, input, output, blueEdges);

        if (contours.size() > 0) {
            MatOfPoint largestCont = contours.stream().reduce((a, b) -> Imgproc.contourArea(a) > Imgproc.contourArea(b) ? a : b).get();
            if (Imgproc.contourArea(largestCont) > 500) {
                Moments p = Imgproc.moments(largestCont, false);

                // Avoiding a divide by zero error by adding 0.001 to the denominator
                x = (int) (p.get_m10() / (p.get_m00() + 0.001));
                y = (int) (p.get_m01() / (p.get_m00() + 0.001));
                Imgproc.putText(output, "Contour center", new org.opencv.core.Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 4);
                telemetry.addData("X position", x);
                telemetry.addData("Y position", y);

                // Reminder: the color scalar is in RGB form
                // Draws line from point to the center (horizontally)
                Imgproc.line(output, new org.opencv.core.Point(x, y), new org.opencv.core.Point(640, y), new org.opencv.core.Scalar(0, 0, 255), 5);

                // Draws line from top center to bottom center (draws vertical center line)
                Imgproc.line(output, new org.opencv.core.Point(640, 0), new org.opencv.core.Point(640, 720), new org.opencv.core.Scalar(255, 0, 0), 2);
            }
        }
        return output;
    }

    public double getCenterError() {
        return 640 - x;
    }

    public LargestContourScanner(Telemetry t) {
        telemetry = t;
    }

}