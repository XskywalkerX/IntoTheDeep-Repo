package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class Robot {
    public DcMotorEx horizontalLinear;
    public DcMotorEx leftLinear;
    public DcMotorEx rightLinear;

    public Servo intakeClaw, intakeX, intakeY, clawB;
    public Servo deliveryClaw, deliveryX, deliveryY;

    public WebcamName webcam;

    public ColorBlobLocatorProcessor colorLocator;
    public VisionPortal portal;



    public Robot(HardwareMap hwMap) {
        webcam = hwMap.get(WebcamName.class, "Webcam 1");

        horizontalLinear = hwMap.get(DcMotorEx.class, "expansion");
        horizontalLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinear = hwMap.get(DcMotorEx.class, "linearEsq");
        //leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLinear = hwMap.get(DcMotorEx.class, "linearDir");
        //rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinear.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeClaw = hwMap.get(Servo.class, "claw");
        clawB = hwMap.get(Servo.class, "clawB");
        intakeX = hwMap.get(Servo.class, "itkX");
        intakeY = hwMap.get(Servo.class, "itkY");

        deliveryClaw = hwMap.get(Servo.class, "clawD");
        deliveryY = hwMap.get(Servo.class, "delY");
        deliveryX = hwMap.get(Servo.class, "delX");

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.9, 0.9, 0.9, -0.9))  // entire frame
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320 * 2, 240 * 2))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(webcam)
                .build();
    }
}
