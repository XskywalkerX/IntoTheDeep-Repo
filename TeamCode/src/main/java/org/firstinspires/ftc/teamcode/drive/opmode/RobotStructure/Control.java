package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.isOn;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.models.Rectangle;

public class Control extends LinearOpMode {

    String s = "INSIDE";

    public static double RECT_X = -23.84;
    public static double RECT_Y = -23.84;
    public static double RECT_WIDTH = 48.5;
    public static double RECT_HEIGHT = 49;

    DriveTrain drive;
    GamepadControl gamepadControl;

    Intake intake;
    Delivery delivery;

    Webcam webcam;
    Pipeline mask;

    enum Mode {
        SEARCHING,
        DRIVER_CONTROL
    }

    Mode mode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        webcam = new Webcam(hardwareMap);

        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        gamepadControl = new GamepadControl();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam.getWebcam(), 30);

        webcam.setPipeline(mask);
        webcam.startStreaming();

        drive.getDrive().setPoseEstimate(drive.lastPose());

        waitForStart();

        Rectangle subArea = new Rectangle(RECT_X, RECT_Y, RECT_WIDTH, RECT_HEIGHT, true);
        subArea.drawRectangle(dashboard);

        while (opModeIsActive()) {

            gamepadControl.movementDriver = gamepad1;
            gamepadControl.driverTwo = gamepad2;

            Pose2d poseEstimate = drive.getDrive().getPoseEstimate();

            s = subArea.contains(poseEstimate.getY(), poseEstimate.getX()) ? "INSIDE" : "OUT";

            switch (mode) {

                case SEARCHING:

                    search();
                    if (gamepad2.x) {
                        intake.catchTeleOp();
                        intake.returnIntake();
                        mode = Mode.DRIVER_CONTROL;
                    }

                    break;

                case DRIVER_CONTROL:

                    drive.gamepadDrive(gamepad1);

                    if (subArea.contains(poseEstimate.getY(), poseEstimate.getX())) {
                        if (gamepad1.b) {
                            mode = Mode.SEARCHING;
                        }
                    }
                    break;
            }

            drive.gamepadDrive(gamepad1);


            if (subArea.contains(poseEstimate.getY(), poseEstimate.getX())) {
                if (gamepad1.b) {
                    mode = Mode.SEARCHING;
                }
            }


            drive.getDrive().update();
            intake.update();
            delivery.update();

            intake.telemetry(telemetry);
            delivery.telemetry(telemetry);
        }
    }

    public void search() {

        if (isOn) {
            gamepadControl.setColorEasy("GREEN");
            drive.getDrive().setDrivePower(new Pose2d(0, 0, Math.toRadians(0)));
        } else {

            gamepadControl.setColorEasy("RED");

            if (mask.getSide().equalsIgnoreCase("LEFT")) {

                drive.getDrive().setWeightedDrivePower(new Pose2d(
                        0,
                        drive.getDrive().getPoseEstimate().getY() - 10,
                        0
                ));

            } else if (mask.getSide().equalsIgnoreCase("RIGHT")) {

                drive.getDrive().setWeightedDrivePower(new Pose2d(
                        0,
                        drive.getDrive().getPoseEstimate().getY() + 10,
                        0
                ));
            }
        }
    }
}