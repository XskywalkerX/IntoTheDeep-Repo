package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Webcam;

@TeleOp
@Config
public class anotherIntakeTest extends LinearOpMode {

    public static double MULTIPLIER = 1;
    Webcam webcam;
    Pipeline colorMask;
    DcMotorEx linear;

    @Override
    public void runOpMode() throws InterruptedException {

        colorMask = new Pipeline();
        webcam = new Webcam(hardwareMap);
        linear = hardwareMap.get(DcMotorEx.class, "linear");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        webcam.setPipeline(colorMask);
        webcam.startStreaming();
        dashboard.startCameraStream(webcam.getWebcam(), 30);

        waitForStart();
        while (opModeIsActive()) {

            double bDistance = colorMask.blueDis();
            double rDistance = colorMask.redDis();

            double move = bDistance < rDistance ? bDistance * MULTIPLIER : rDistance * MULTIPLIER;

            linear.setTargetPosition((int) move);

            double servoMove = linear.getCurrentPosition() / move;

            //servo.setPosition(move);

            linear.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("TARGET", linear.getTargetPosition());
            telemetry.addData("TICKS", linear.getCurrentPosition());
            telemetry.addData("BLUE DISTANCE", colorMask.blueDis());
            telemetry.addData("RED DISTANCE", colorMask.redDis());
            telemetry.addData("YELLOW DISTANCE", colorMask.yellowDis());
            telemetry.update();
        }
    }
}
