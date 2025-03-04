package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.systems.ArmIntakeSystem;
import org.firstinspires.ftc.teamcode.systems.GamepadBoladao;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.List;

@Config
@Autonomous
public class VisionTest extends LinearOpMode {

    PIDController intakeXPID;
    PIDController linearPID;

    public static double Kp_itkX = -0.00005;
    public static double Ki_itkX = 0.0001;
    public static double Kd_itkX = 0.000001;

    public static double linear_max_vel = 0.2;
    public static double Kp_linear = 0.0015;
    public static double Ki_linear = 0;
    public static double Kd_linear = 0.00001;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        GamepadBoladao gamepadBoladao = new GamepadBoladao(gamepad1);
        intakeXPID = new PIDController(Kp_itkX, Ki_itkX, Kd_itkX);
        linearPID = new PIDController(Kp_linear, Ki_linear, Kd_linear);

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.9, 0.9, 0.9, -0.9))  // entire frame
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320 * 2, 240 * 2))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(robot.webcam)
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.


        while (opModeIsActive() || opModeInInit()) {
            gamepadBoladao.readGamepad(gamepad1);
            double theta = 0;
            intakeXPID.setP(Kp_itkX);
            intakeXPID.setI(Ki_itkX);
            intakeXPID.setD(Kd_itkX);

            linearPID.setP(Kp_linear);
            linearPID.setI(Ki_linear);
            linearPID.setD(Kd_linear);


            telemetry.addData("preview on/off", "... Camera Stream\n");
            telemetry.addData("fps", portal.getFps());

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(1500, 500000, blobs);  // filter out very small blobs.

            telemetry.addLine(" Area Density Aspect  Center");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                Moments mu = Imgproc.moments(b.getContour());

                // Compute orientation
                double aa = mu.mu20 / mu.m00;
                double bb = mu.mu11 / mu.m00;
                double cc = mu.mu02 / mu.m00;
                theta = Math.atan2(2 * bb, aa - cc) * 180 / Math.PI;
                telemetry.addData("Theta:", theta);
            }

            if (gamepad1.a) {
                robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                robot.intakeY.setPosition(Globals.INTAKE_Y_READ);
                robot.clawB.setPosition(0);
            }

            if (gamepad1.left_bumper) {
                if (!blobs.isEmpty()) {
                    double power = linearPID.calculate(blobs.get(0).getBoxFit().center.y, Globals.CAMERA_Y_CATCH_SETPOINT);

                    if (power > linear_max_vel) {
                        power = linear_max_vel;
                    } else if (power < -linear_max_vel) {
                        power = -linear_max_vel;
                    }

                    robot.horizontalLinear.setPower(power);
                    telemetry.addData("Horizontal Linear Power:", power);
                } else {
                    robot.horizontalLinear.setPower(0);
                }
            } else {
                robot.horizontalLinear.setPower(0);
            }

            if (gamepadBoladao.ONEwasBPressed()) {
                robot.clawB.setPosition(normalize(Math.abs(theta)));
                telemetry.addData("Claw B Set Position ", robot.clawB.getPosition());
            }

            if (gamepad1.x) {
                robot.clawB.setPosition(0);
            }

            if (gamepad1.right_bumper) {
                if (!blobs.isEmpty()) {
                    double servoOutputPosition = robot.intakeX.getPosition()
                            + intakeXPID.calculate(blobs.get(0).getBoxFit().center.x, Globals.CAMERA_X_CATCH_SETPOINT);

                    telemetry.addData("Intake X Output Position:", servoOutputPosition);
                    telemetry.addData("Intake X get Position:", robot.intakeX.getPosition());

                    if (servoOutputPosition < 0) {
                        servoOutputPosition = 0;
                    } else if (servoOutputPosition > 1) {
                        servoOutputPosition = 1;
                    }

                    if (robot.intakeX.getPosition() < Globals.INTAKE_X_SAFE_MIN) {
                        robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MIN);
                    } else if (robot.intakeX.getPosition() > Globals.INTAKE_X_SAFE_MAX) {
                        robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MAX);
                    }

                    robot.intakeX.setPosition(servoOutputPosition);
                } else {
                    robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                    robot.intakeY.setPosition(Globals.INTAKE_Y_READ);
                }
            }
            telemetry.update();
            sleep(50);

        }
    }

    double normalize(double value) {
        return (value) / 180;
    }

    double normalizeX(double value) {
        double x = value > Globals.SERVO_MAX ? 180 - value : value;

        return (x - Globals.SERVO_MIN) / (Globals.SERVO_MAX - Globals.SERVO_MIN);
    }
}
