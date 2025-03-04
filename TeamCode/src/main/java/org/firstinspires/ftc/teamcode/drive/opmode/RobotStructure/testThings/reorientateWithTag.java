package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
@Disabled
public class reorientateWithTag extends LinearOpMode {

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 5, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -90, 0, 0);

    SampleMecanumDrive drive;
    AprilTagProcessor processor;
    VisionPortal vp;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        vp = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(vp, 30);

        waitForStart();
        while (opModeIsActive()) {

            drive.update();

            if (!processor.getDetections().isEmpty()) {

                List<AprilTagDetection> currentDetections = processor.getDetections();

                for (AprilTagDetection tag : currentDetections) {

                    if (tag.robotPose != null) {
                        double x = tag.robotPose.getPosition().x;
                        double y = tag.robotPose.getPosition().y;
                        double yaw = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                        drive.setPoseEstimate(new Pose2d(x, y, Math.toRadians(yaw)));

                        telemetry.addData("X", tag.ftcPose.x);
                        telemetry.addData("Z", tag.ftcPose.z);
                        telemetry.addData("HEADING", tag.ftcPose.bearing);
                    }
                }
            }
        }
    }
}
