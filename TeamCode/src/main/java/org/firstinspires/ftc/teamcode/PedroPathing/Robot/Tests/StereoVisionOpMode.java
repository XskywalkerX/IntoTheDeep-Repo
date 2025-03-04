package org.firstinspires.ftc.teamcode.PedroPathing.Robot.Tests;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfByte;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.concurrent.atomic.AtomicReference;

@SuppressLint({"UnsafeDynamicallyLoadedCode", "SdCardPath"})
@TeleOp
@Disabled
public class StereoVisionOpMode extends LinearOpMode {
    OpenCvWebcam leftWebcam, rightWebcam;

    static {
        System.loadLibrary("gcc_deps");
        System.loadLibrary("opencv_core");
        System.loadLibrary("opencv_highgui");
        System.loadLibrary("opencv_videoio");
        System.loadLibrary("opencv_imgcodecs");
        System.loadLibrary("opencv_imgproc");
        System.loadLibrary("opencv_calib3d");
        System.loadLibrary("stereoVision");
    }

    public native float processStereoImages(byte[] leftFrame, byte[] rightFrame);

    @Override
    public void runOpMode() {
        // Initialize webcams
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        leftWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "LeftWebcam"), cameraMonitorViewId);
        rightWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RightWebcam"), cameraMonitorViewId);

        // AtomicReferences to hold frames
        AtomicReference<byte[]> leftFrame = new AtomicReference<>();
        AtomicReference<byte[]> rightFrame = new AtomicReference<>();

        leftWebcam.setPipeline(new FrameCapturePipeline(leftFrame));
        rightWebcam.setPipeline(new FrameCapturePipeline(rightFrame));

        leftWebcam.openCameraDevice();
        rightWebcam.openCameraDevice();

        leftWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        rightWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            byte[] leftImage = leftFrame.get();
            byte[] rightImage = rightFrame.get();

            if (leftImage != null && rightImage != null) {
                // Process frames using JNI
                float distance = processStereoImages(leftImage, rightImage);

                telemetry.addData("Distance (cm)", distance);
                telemetry.update();
            } else {
                telemetry.addData("Waiting for frames", "...");
                telemetry.update();
            }

            sleep(50); // Adjust sleep time as needed for performance
        }

        // Stop cameras after the OpMode ends
        leftWebcam.stopStreaming();
        rightWebcam.stopStreaming();
    }

    static class FrameCapturePipeline extends OpenCvPipeline {
        private final AtomicReference<byte[]> frameRef;

        FrameCapturePipeline(AtomicReference<byte[]> frameRef) {
            this.frameRef = frameRef;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Encode frame to byte array for JNI processing
            MatOfByte encodedImage = new MatOfByte();
            Imgcodecs.imencode(".jpg", input, encodedImage);

            // Convert MatOfByte to byte array
            frameRef.set(encodedImage.toArray());

            return input; // Display frame unaltered
        }
    }
}