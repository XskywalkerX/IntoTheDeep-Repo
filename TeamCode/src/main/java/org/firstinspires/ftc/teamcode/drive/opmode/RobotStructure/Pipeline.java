package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


@Config
public class Pipeline extends OpenCvPipeline {

    public static boolean blueSide = false;

    public static double blueSampleArea = 0;
    public static double redSampleArea = 0;
    public static double yellowSampleArea = 0;

    public static double redArea = 0;
    public static double blueArea = 0;
    public static double yellowArea = 0;

    public static Point centroid;
    public static Point redCentroid;
    public static Point yellowCentroid;

    public static double p1x = 240;
    public static double p1y = 140;
    public static double rectWidth = 180;
    public static double rectHeight = 280;

    volatile public static String side = "IDLE";

    volatile public static boolean isOn = false;
    volatile public static boolean redOn = false;
    volatile public static boolean yellowOn = false;

    public static double cX = 0;
    public static double cY = 0;

    public static double cX2 = 0;
    public static double cY2 = 0;

    public static double cX3 = 0;
    public static double cY3 = 0;

    public static double width = 0;
    public static double width2 = 0;
    public static double width3 = 0;

    //BLUE FILTER
    public static double lowerBlue = 5;
    public static double lowerGreen = 230;
    public static double lowerRed = 130;

    public static double upperBlue = 15;
    public static double upperGreen = 255;
    public static double upperRed = 170;


    //RED FILTER
    public static double lowerBlue2 = 120;
    public static double lowerGreen2 = 90;
    public static double lowerRed2 = 135;

    public static double upperBlue2 = 130;
    public static double upperGreen2 = 235;
    public static double upperRed2 = 195;


    //YELLOW FILTER
    public static double lowerBlue3 = 90;
    public static double lowerGreen3 = 90;
    public static double lowerRed3 = 140;

    public static double upperBlue3 = 105;
    public static double upperGreen3 = 240;
    public static double upperRed3 = 245;


    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75; // Replace with the actual width of the object in real-world units
    public static final double focalLength = 377.95; // Replace with the focal length of the camera in pixels


    @Override
    public Mat processFrame(Mat input) {

        // Preprocess the frame to detect color regions
        Mat colorMask = preprocessFrame(input, "blue");
        Mat redMask = preprocessFrame(input, "red");
        Mat yellowMask = preprocessFrame(input, "yellow");

        Mat myMask = new Mat();
        Core.bitwise_or(colorMask, redMask, myMask);
        Core.bitwise_or(myMask, yellowMask, myMask);

        // Find contours of the detected color regions
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();

        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redMask, redContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowMask, yellowContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            MatOfPoint largestContour = findLargestContour(contours);

            double biggerArea = Imgproc.contourArea(largestContour);
            blueArea = biggerArea;

            Imgproc.drawContours(input, Collections.singletonList(largestContour), -1, new Scalar(255, 0, 0), 2); // Blue color
            width = calculateWidth(largestContour);
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            MatOfPoint2f contour2f = new MatOfPoint2f();
            largestContour.convertTo(contour2f, CvType.CV_32F);
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Point[] points = new Point[4];
            rotatedRect.points(points);

            // Draw the rotated rectangle
            for (int i = 0; i < points.length; i++) {
                Point pt1 = points[i];
                Point pt2 = points[(i + 1) % points.length];
                Imgproc.line(input, pt1, pt2, new Scalar(0, 255, 0), 2); // Draw line with green color and thickness 2
            }

            blueSampleArea = rotatedRect.size.area();

            labelContour(new Scalar(0, 255, 0), input, largestContour, "BLUE", biggerArea);  // Label as "BLUE"

        }

        // Draw red contours and label them
        for (MatOfPoint contour : redContours) {
            double area = Imgproc.contourArea(contour);

            MatOfPoint largestContour = findLargestContour(redContours);

            double biggerArea = Imgproc.contourArea(largestContour);

            redArea = biggerArea;

            Imgproc.drawContours(input, Collections.singletonList(largestContour), -1, new Scalar(0, 0, 255), 2); // Red color
            width2 = calculateWidth(largestContour);
            Moments moments = Imgproc.moments(largestContour);
            cX2 = moments.get_m10() / moments.get_m00();
            cY2 = moments.get_m01() / moments.get_m00();

            MatOfPoint2f contour2f = new MatOfPoint2f();
            largestContour.convertTo(contour2f, CvType.CV_32F);
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Point[] points = new Point[4];
            rotatedRect.points(points);

            // Draw the rotated rectangle
            for (int i = 0; i < points.length; i++) {
                Point pt1 = points[i];
                Point pt2 = points[(i + 1) % points.length];
                Imgproc.line(input, pt1, pt2, new Scalar(0, 0, 255), 2); // Draw line with green color and thickness 2
            }
            redSampleArea = rotatedRect.size.area();

            labelContour(new Scalar(0, 0, 255), input, largestContour, "RED", biggerArea);  // Label as "RED"

        }

        // Draw yellow contours and label them
        for (MatOfPoint contour : yellowContours) {
            double area = Imgproc.contourArea(contour);

            MatOfPoint largestContour = findLargestContour(yellowContours);

            double biggerArea = Imgproc.contourArea(largestContour);
            yellowArea = biggerArea;

            Imgproc.drawContours(input, Collections.singletonList(largestContour), -1, new Scalar(0, 255, 0), 2); // Yellow color
            width3 = calculateWidth(largestContour);
            Moments moments = Imgproc.moments(largestContour);
            cX3 = moments.get_m10() / moments.get_m00();
            cY3 = moments.get_m01() / moments.get_m00();

            MatOfPoint2f contour2f = new MatOfPoint2f();
            largestContour.convertTo(contour2f, CvType.CV_32F);
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Point[] points = new Point[4];
            rotatedRect.points(points);

            // Draw the rotated rectangle
            for (int i = 0; i < points.length; i++) {
                Point pt1 = points[i];
                Point pt2 = points[(i + 1) % points.length];
                Imgproc.line(input, pt1, pt2, new Scalar(255, 0, 0), 2); // Draw line with green color and thickness 2
            }

            yellowSampleArea = rotatedRect.size.area();

            labelContour(new Scalar(255, 0, 0), input, largestContour, "YELLOW", biggerArea);  // Label as "YELLOW"

        }
        // Find the largest color contour (blob)

            /*if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();*/

        centroid = new Point(cX, cY);
        redCentroid = new Point(cX2, cY2);
        yellowCentroid = new Point(cX3, cY3);

        Rect rect = new Rect(new Point(p1x, p1y), new Size(rectWidth, rectHeight));

        Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 0));
        // Draw a dot at the centroid
        String label = "(" + (int) cX + ", " + (int) cY + ")";
        Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        isOn = rect.contains(centroid);
        redOn = rect.contains(redCentroid);
        yellowOn = rect.contains(yellowCentroid);

        if (centroid.x > rect.x) {
            side = "left";
        } else if (centroid.x < rect.x) {
            side = "right";
        }

        return input;

    }

    private void labelContour(Scalar labelColor, Mat input, MatOfPoint contour, String colorLabel, double area) {
        Moments moments = Imgproc.moments(contour);
        double cX = moments.get_m10() / moments.get_m00();
        double cY = moments.get_m01() / moments.get_m00();

        Imgproc.circle(input, new Point(cX + 10, cY), 5, labelColor, -1);

        // Label the contour with its color name and area
        String label = colorLabel + ": " + (int) area + " px²";
        Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, labelColor, 2);
    }

    private Mat preprocessFrame(Mat frame, String color) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerColor, upperColor;

        // Define HSV ranges for each color
        switch (color) {
            case "blue":
                lowerColor = new Scalar(lowerBlue, lowerGreen, lowerRed);
                upperColor = new Scalar(upperBlue, upperGreen, upperRed);
                break;
            case "red":
                lowerColor = new Scalar(lowerBlue2, lowerGreen2, lowerRed2);
                upperColor = new Scalar(upperBlue2, upperGreen2, upperRed2);
                break;
            case "yellow":
                lowerColor = new Scalar(lowerBlue3, lowerGreen3, lowerRed3);
                upperColor = new Scalar(upperBlue3, upperGreen3, upperRed3);
                break;
            default:
                throw new IllegalArgumentException("Unsupported color: " + color);
        }

        Mat colorMask = new Mat();
        Core.inRange(hsvFrame, lowerColor, upperColor, colorMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

        return colorMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
            blueArea = area;
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }

    public boolean getIsON() {
        return isOn;
    }

    public String getSide() {
        return side;
    }
}