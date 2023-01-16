package org.firstinspires.ftc.teamcode.drive.vision;

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

public class ColorScanner extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat edges = new Mat();
    Mat output = new Mat();

    Telemetry telemetry;

    // List<MatOfPoint> contours = new ArrayList<>();
    int largestMarker = 0;

    Scalar lowerColor = new Scalar(100, 70, 150);
    Scalar upperColor = new Scalar(115, 255, 255);

    // HSV value for blue
    Scalar blueLower = new Scalar(100, 75, 0);
    Scalar blueUpper = new Scalar(120, 255, 255);

    // HSV for yellow
    Scalar yellowLower = new Scalar(20, 120, 0);
    Scalar yellowUpper = new Scalar(30, 255, 255);

    // HSV for red, yellow, and blue (subtract blue and yellow to get red)
    Scalar rybLower = new Scalar(0, 90, 0);
    Scalar rybUpper = new Scalar(255, 255, 255);


    @Override
    public Mat processFrame(Mat input) {
        // Resetting all of the variables since the last call to this method
        output.setTo(new Scalar(0, 0, 0));

        // Converts image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Detects alliance color
        // Red has two ranges on opposite ends of the color spectrum
        Core.inRange(hsv, lowerColor, upperColor, edges);


        // Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create output matrix
        Core.bitwise_and(input, input, output, edges);
        return output;

    }

    public void setUpperColor(double h, double s, double v) {
        upperColor = new Scalar(h, s, v);
    }

    public void setLowerColor(double h, double s, double v) {
        lowerColor = new Scalar(h, s, v);
    }

    public ColorScanner(Telemetry t) {
        telemetry = t;
    }

}