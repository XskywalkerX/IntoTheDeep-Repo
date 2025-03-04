package org.firstinspires.ftc.teamcode.RobotStructure.Gustavo;

import android.media.MediaRecorder;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotStructure.Webcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
@Disabled
public class GustavoTag extends LinearOpMode {
    AprilTagProcessor TagProcessor;
    VisionPortal visaoportal;

    @Override
    public void runOpMode() throws InterruptedException {
        TagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .build();
        visaoportal = new VisionPortal.Builder()
                .addProcessor(TagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()){
            if (!TagProcessor.getDetections().isEmpty()){
                AprilTagDetection tag = TagProcessor.getDetections().get(0);

                telemetry.addData("X RELAÇÃO ROBÔ", tag.ftcPose.x);
                telemetry.addData("Y RELAÇÃO ROBÔ", tag.ftcPose.y);
                telemetry.addData("Z RELAÇÃO ROBÔ", tag.ftcPose.z);
                telemetry.update();
            }
        }
    }
}