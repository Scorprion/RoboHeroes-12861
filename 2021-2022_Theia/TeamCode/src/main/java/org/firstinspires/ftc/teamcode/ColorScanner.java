package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Point;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ColorScanner extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat blueEdges = new Mat();
    Mat markerEdges = new Mat();
    Mat combinedEdges = new Mat();
    Mat output = new Mat();

    Telemetry telemetry;

    List<MatOfPoint> barcode = new ArrayList<>();
    List<MatOfPoint> marker = new ArrayList<>();
    List<Point> centers = new ArrayList<>();

    List<MatOfPoint> largestBars = new ArrayList<>();
    int largestMarker = 0;

    Scalar lowerBlue = new Scalar(100, 70, 0);
    Scalar upperBlue = new Scalar(115, 255, 255);

    Scalar lowerGreen = new Scalar(45, 75, 0);
    Scalar upperGreen = new Scalar(100, 255, 255);

    enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    Location scan = Location.UNKNOWN;


    @Override
    public Mat processFrame(Mat input) {
        // Resetting all of the variables since the last call to this method
        output.setTo(new Scalar(0, 0, 0));
        largestBars.clear();
        centers.clear();
        barcode.clear();
        marker.clear();
        largestMarker = 0;

        // Converts image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Detects Blue
        Core.inRange(hsv, lowerBlue, upperBlue, blueEdges);
        Imgproc.findContours(blueEdges, barcode, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Detects marker color
        Core.inRange(hsv, lowerGreen, upperGreen, markerEdges);
        Imgproc.findContours(markerEdges, marker, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create output matrix
        Core.add(markerEdges, blueEdges, combinedEdges);
        Core.bitwise_and(input, input, output, combinedEdges);

        // List of the largest 2 contours detected from the inRange on the barcode
        List<Moments> mu = new ArrayList<>(2);
        if (marker.size() > 0 && barcode.size() > 1) {

            // Find the two largest by first adding the first two contours and replacing them if there exists another larger contour
            if(Imgproc.contourArea(barcode.get(0)) > Imgproc.contourArea(barcode.get(1))) {
                largestBars.add(barcode.get(0));
                largestBars.add(barcode.get(1));
            } else {
                largestBars.add(barcode.get(1));
                largestBars.add(barcode.get(0));
            }

            for(int i = 2; i < barcode.size(); i++) {
                if(Imgproc.contourArea(barcode.get(i)) > Imgproc.contourArea(largestBars.get(1))) {
                    if(Imgproc.contourArea(barcode.get(i)) > Imgproc.contourArea(largestBars.get(0))) {
                        largestBars.add(0, barcode.get(i));
                        largestBars.remove(2);
                    } else {
                        largestBars.set(1, barcode.get(i));
                    }
                }
            }

            for (int i = 0; i < largestBars.size(); i++) {
                mu.add(i, Imgproc.moments(largestBars.get(i), false));
                Moments p = mu.get(i);

                // Avoiding a divide by zero error by adding 0.01 to the denominator
                int x = (int) (p.get_m10() / (p.get_m00() + 0.01));
                int y = (int) (p.get_m01() / (p.get_m00() + 0.01));
                centers.add(new Point(x, y));
                Imgproc.putText(output, "Contour center" + i, new org.opencv.core.Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 4);
            }

            // Add midpoint to the list of contour centers
            centers.add((centers.get(0).plus(centers.get(1))).div(2.0));
            Imgproc.putText(output, "Midpoint", new org.opencv.core.Point(centers.get(2).getX(), centers.get(2).getY()), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 4);

            // Sort the centers list based on their x value
            centers.sort(Comparator.comparing(Point::getX));

            for(int i = 1; i < marker.size(); i++) {
                if(Imgproc.contourArea(marker.get(i)) > Imgproc.contourArea(marker.get(largestMarker))) {
                    largestMarker = i;
                }
            }
            Moments mark = Imgproc.moments(marker.get(largestMarker), false);
            Point markerPoint = new Point((int) (mark.get_m10() / mark.get_m00()), (int) (mark.get_m01() / mark.get_m00()));
            Imgproc.putText(output, "Marker", new org.opencv.core.Point(markerPoint.getX(), markerPoint.getY()), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 4);
            double minDist = Double.POSITIVE_INFINITY;
            int argmin = -1;
            for (int i = 0; i < centers.size(); i++) {
                double currentDist = markerPoint.dist(centers.get(i));
                telemetry.addData("Current dist", currentDist);
                if (minDist > currentDist) {
                    minDist = currentDist;
                    argmin = i;
                }
            }
            telemetry.update();

            switch(argmin) {
                case -1:
                    scan = Location.UNKNOWN;
                    break;
                case 0:
                    scan = Location.LEFT;
                    break;
                case 1:
                    scan = Location.MIDDLE;
                    break;
                case 2:
                    scan = Location.RIGHT;
                    break;
                default:
                    break;
            }
        }

        return output;
    }
    public ColorScanner(Telemetry t) {
        telemetry = t;
    }
    public Location getLocation() { return scan; }

}
