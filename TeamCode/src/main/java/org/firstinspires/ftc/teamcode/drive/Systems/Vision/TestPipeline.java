package org.firstinspires.ftc.teamcode.drive.Systems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Systems.InverseKinematics.IK;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@TeleOp
public class TestPipeline extends LinearOpMode {

    OpenCvWebcam webcam;
    SamplePipeline pip;

    Servo clawB, armX, armY;

    IK kinematics;

    @Override
    public void runOpMode() throws InterruptedException {

        clawB = hardwareMap.get(Servo.class, "clawB");
        armX = hardwareMap.get(Servo.class, "armX");
        armY = hardwareMap.get(Servo.class, "armY");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        pip = new SamplePipeline();

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pip);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        dashboard.startCameraStream(webcam, 30);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running...");
            telemetry.addData("THETA", pip.SampleAngle());
            telemetry.addData("POSITION", Arrays.toString(pip.coordinate()));
            telemetry.addData("SERVO", clawB.getPosition());
            telemetry.update();

            clawB.setPosition(normalize(pip.SampleAngle()));
        }
    }

    double normalize(double value) {
        return (value + 90) / 180;
    }
}
