package org.firstinspires.ftc.teamcode.PedroPathing.Robot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Systems.Vision.SamplePipeline;
import org.firstinspires.ftc.teamcode.RobotStructure.PIDController.PIDController;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp(name = "TEST INTAKE SYSTEM")
public class testExpansion extends LinearOpMode {

    SamplePipeline pip;

    Servo intakeX, intakeY;
    DcMotorEx motor;
    Servo clawB;

    public static double MAX_POWER = 0.13;
    public static double resetPos = 0.4;
    public static double resetPosPhi = 0.4;

    public static double POS = 0.3;
    public static double kp = 0.002;
    public static double kd = 2;
    public static double ki = 0;

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "expansion");
        intakeY = hardwareMap.get(Servo.class, "itkY");
        intakeX = hardwareMap.get(Servo.class, "itkX");

        clawB = hardwareMap.get(Servo.class, "clawB");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeX.setDirection(Servo.Direction.REVERSE);

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

        intakeY.setPosition(POS);

        PIDController controller = new PIDController(kp, ki, kd);

        waitForStart();
        while(opModeIsActive()) {
            controller.setKp(kp);
            controller.setKd(kd);
            controller.setKi(ki);
            controller.setTargetPosition(0);
            controller.setCurrentPosition(pip.SampleY());

            double yawAngle = 1/(Math.tan(pip.SampleX() / pip.SampleY()));
            double motorPower = -controller.dice();

            if (gamepad1.a) {
                if (motorPower >= MAX_POWER) {
                    motor.setPower(MAX_POWER);
                } else if (motorPower <= -MAX_POWER) {
                    motor.setPower(-MAX_POWER);
                } else {
                    motor.setPower(motorPower);
                }
            }

            if (gamepad1.b) {
                clawB.setPosition(normalize(Math.abs(pip.SampleAngle())));
            }
            if (gamepad1.y) {
                clawB.setPosition(resetPos);
            }
            if (gamepad1.dpad_left) {
                intakeX.setPosition(normalizeX(pip.PhiAngle()));
            } else if (gamepad1.dpad_right) {
                intakeX.setPosition(resetPosPhi);
            }

            telemetry.addData("POWER", motor.getPower());
            telemetry.addData("Claw B Position", clawB.getPosition());
            telemetry.addData("Intake X Position", intakeX.getPosition());
            telemetry.addData("Phi Angle", pip.PhiAngle());
            telemetry.update();
        }
    }

    double normalize(double value) {
        return (value) / 180;
    }

    double normalizeX(double value) {
        double x = value > Globals.SERVO_MAX ? 180 - value : value;
        double y = (x - Globals.SERVO_MIN) / (Globals.SERVO_MAX - Globals.SERVO_MIN);

        return y;
    }
}
