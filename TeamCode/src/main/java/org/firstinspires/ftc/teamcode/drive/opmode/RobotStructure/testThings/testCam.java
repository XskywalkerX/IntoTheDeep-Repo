package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.cX;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.cY;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.getDistance;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.width;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Webcam;

@TeleOp
public class testCam extends LinearOpMode {

    Webcam webcam;
    Pipeline colormask;

    @Override
    public void runOpMode() throws InterruptedException {

        colormask = new Pipeline();

        webcam = new Webcam(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam.getWebcam(), 30);

        webcam.setPipeline(colormask);

        webcam.startStreaming();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }
    }
}
