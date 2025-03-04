package org.firstinspires.ftc.teamcode.drive.Systems.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class SamplePipeline extends OpenCvPipeline {

    public static double CAMERA_OFFSET = 0;

    public static double lbRed = 100;
    public static double lbGreen = 150;
    public static double lbBlue = 50;

    public static double ubRed = 140;
    public static double ubGreen = 255;
    public static double ubBlue = 255;


    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private double cx = -1, cy = -1, theta = 0, phi = 0;
    private double cxTransformed = 0, cyTransformed = 0;

    @Override
    public Mat processFrame(Mat input) {
        int centerX = input.width() / 2;
        int centerY = input.height() / 2;

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(lbRed, lbGreen, lbBlue);
        Scalar upperBlue = new Scalar(ubRed, ubGreen, ubBlue);
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                Moments mu = Imgproc.moments(largestContour);
                if (mu.m00 != 0) {
                    cx = mu.m10 / mu.m00;
                    cy = mu.m01 / mu.m00;

                    // Transform to center-based coordinates
                    cxTransformed = cx - centerX;
                    cyTransformed = centerY - cy + CAMERA_OFFSET;

                    // Compute orientation
                    double a = mu.mu20 / mu.m00;
                    double b = mu.mu11 / mu.m00;
                    double c = mu.mu02 / mu.m00;
                    theta = 0.5 * Math.atan2(2 * b, a - c) * 180 / Math.PI;

                    // Compute angle between the detected object and camera center
                    phi = (Math.atan2(cyTransformed, cxTransformed) * 180 / Math.PI) + 90;

                    // Draw the largest contour and centroid
                    Imgproc.drawContours(input, Collections.singletonList(largestContour), -1, new Scalar(0, 255, 0), 2);
                    Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 0, 255), -1);
                    Imgproc.putText(input, "Angle: " + String.format("%.2f", theta) + " deg", new Point(10, 50),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 0, 255), 2);
                    Imgproc.putText(input, "Center: (" + (int)cxTransformed + ", " + (int)cyTransformed + ")",
                            new Point(10, 80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 0, 255), 2);
                    Imgproc.putText(input, "Phi: " + String.format("%.2f", phi) + " deg", new Point(10, 110),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 0, 0), 2);
                }
            }
        }
        return input;
    }


    public double SampleAngle() {
        return theta;
    }

    public double PhiAngle() {
        return phi;
    }

    public double SampleX() {
        return cxTransformed;
    }

    public double SampleY() {
        return cyTransformed;
    }

    public double[] coordinate() {
        return new double[]{cxTransformed, cyTransformed};
    }
}
