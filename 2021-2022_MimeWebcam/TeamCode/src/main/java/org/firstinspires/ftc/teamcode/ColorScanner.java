package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ColorScanner extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat blueEdges = new Mat();
    Mat markerEdges = new Mat();
    Mat combinedEdges = new Mat();

    List<MatOfPoint> barcode = new ArrayList<>();
    List<MatOfPoint> marker = new ArrayList<>();
    List<Point> centers = new ArrayList<>();

    Scalar lowerBlue = new Scalar(100, 100, 75);
    Scalar upperBlue = new Scalar(120, 255, 255);

    Scalar lowerWheelGreen = new Scalar(40, 100, 100);
    Scalar upperWheelGreen = new Scalar(60, 255, 255);

    enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    Location scan = Location.UNKNOWN;


    @Override
    public Mat processFrame(Mat input) {
        // Converts image to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Detects Blue (just blue for testing)
        Core.inRange(hsv, lowerBlue, upperBlue, blueEdges);
        Imgproc.findContours(blueEdges, barcode, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Detects marker color (a compliant wheel for testing)
        Core.inRange(hsv, lowerWheelGreen, upperWheelGreen, markerEdges);
        Imgproc.findContours(markerEdges, marker, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


        List<Moments> mu = new ArrayList<>(barcode.size());

        if (marker.size() > 0 && centers.size() > 0) {
            // Add all contours detected from the inRange
            for (int i = 0; i < barcode.size(); i++) {
                mu.add(i, Imgproc.moments(barcode.get(i), false));
                Moments p = mu.get(i);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());
                centers.add(new Point(x, y));
            }

            // Add midpoint to the list of contour centers
            centers.add(centers.get(0).plus(centers.get(1).div(2.0)));

            // Sort the centers list based on their x value
            Collections.sort(centers, (c1, c2) ->  c1.getX() < c2.getX() ? 0 : 1 );

            Moments mark = Imgproc.moments(marker.get(0), false);
            Point markerPoint = new Point((int) (mark.get_m10() / mark.get_m00()), (int) (mark.get_m01() / mark.get_m00()));
            double minDist = Double.POSITIVE_INFINITY;
            int argmin = -1;
            for (int i = 0; i < centers.size(); i++) {
                double currentDist = markerPoint.dist(centers.get(i));
                if (minDist > currentDist) {
                    minDist = currentDist;
                    argmin = i;
                }
            }

            switch(argmin) {
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

        Core.add(blueEdges, markerEdges, combinedEdges);
        hsv.release();
        markerEdges.release();
        return blueEdges;
    }

    public Location getLocation() { return scan; }
}
