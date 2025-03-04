package org.firstinspires.ftc.teamcode.drive.Systems.JNI;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotStructure.Pipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@SuppressLint("UnsafeDynamicallyLoadedCode")
@TeleOp
@Disabled
public class JNIOpMode extends LinearOpMode {

    OpenCvWebcam webcam;

    static {
        System.loadLibrary("NativeProcessor");
    }

    public static native void processFrame(long matAddr);

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set frame processing callback
        webcam.setPipeline(new Pipeline() {
            @Override
            public Mat processFrame(Mat input) {
                // Convert to grayscale before sending to JNI (optional)
                Mat processed = new Mat();
                Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGBA2GRAY);

                // Pass Mat to JNI
                JNIOpMode.processFrame(processed.getNativeObjAddr());

                return processed; // Display processed frame
            }
        });

        // Start streaming
        webcam.openCameraDevice();
        webcam.startStreaming(640, 480);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Status", "Running...");
            telemetry.update();
        }
    }
}
