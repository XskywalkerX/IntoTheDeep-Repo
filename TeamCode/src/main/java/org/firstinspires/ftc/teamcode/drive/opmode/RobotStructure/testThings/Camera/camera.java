package org.firstinspires.ftc.teamcode.RobotStructure.testThings.Camera;

import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.blueArea;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.blueSampleArea;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.cX;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.cY;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.getDistance;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.isOn;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.redArea;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.redSampleArea;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.side;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.width;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.yellowArea;
import static org.firstinspires.ftc.teamcode.RobotStructure.Pipeline.yellowSampleArea;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotStructure.Pipeline;
import org.firstinspires.ftc.teamcode.RobotStructure.Webcam;
import org.firstinspires.ftc.teamcode.models.Rectangle;

@Config
@TeleOp(name = "blablabla")
@Disabled
public class camera extends LinearOpMode {

    String s = "INSIDE";

    public static double RECT_X = -23.84;
    public static double RECT_Y = -23.84;
    public static double RECT_WIDTH = 48.5;
    public static double RECT_HEIGHT = 49;

    public static boolean isSearching = false;

    Webcam webcam;
    Pipeline colormask;

    SampleMecanumDrive drive;

    public static MODE mode = MODE.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        colormask = new Pipeline();

        webcam = new Webcam(hardwareMap);
        webcam.setPipeline(colormask);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam.getWebcam(), 30);

        webcam.setPipeline(colormask);

        webcam.startStreaming();

        waitForStart();

        while (opModeIsActive()) {

            isSearching = mode == MODE.SEARCHING;

            Pose2d poseEstimate = drive.getPoseEstimate();

            Rectangle subArea = new Rectangle(RECT_X, RECT_Y, RECT_WIDTH, RECT_HEIGHT, true);
            subArea.drawRectangle(dashboard);


            switch (mode) {

                case SEARCHING:

                    search();
                    if (gamepad1.x) {
                        //moveIntake
                        mode = MODE.DRIVER_CONTROL;
                    }

                    break;

                case DRIVER_CONTROL:

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (subArea.contains(poseEstimate.getY(), poseEstimate.getX())) {
                        if (gamepad1.b) {
                            mode = MODE.SEARCHING;
                        }
                    }

                    break;
            }

            drive.update();

            s = subArea.contains(poseEstimate.getY(), poseEstimate.getX()) ? "INSIDE" : "OUT";

            telemetry.addLine(s);
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addLine(colormask.getIsON() ? "ON" : "OFF");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.addData("MODE", mode == MODE.SEARCHING ? "SEARCHING" : "DRIVER_CONTROLLED");
            telemetry.addData("BLUE AREA", blueSampleArea);
            telemetry.addData("RED AREA", redSampleArea);
            telemetry.addData("YELLOW AREA", yellowSampleArea);
            telemetry.addData("BLUE", blueArea);
            telemetry.addData("RED", redArea);
            telemetry.addData("YELLOW", yellowArea);
            telemetry.addLine(side);
            telemetry.update();
        }
    }

    public void search() {

        if (isOn) {
            drive.setDrivePower(new Pose2d(0, 0, Math.toRadians(0)));
        } else {

            if (colormask.getSide().equalsIgnoreCase("LEFT")) {

                drive.setWeightedDrivePower(new Pose2d(
                        0,
                        drive.getPoseEstimate().getY() - 10,
                        0
                ));

            } else if (colormask.getSide().equalsIgnoreCase("RIGHT")) {

                drive.setWeightedDrivePower(new Pose2d(
                        0,
                        drive.getPoseEstimate().getY() + 10,
                        0
                ));
            }
        }
    }
}
