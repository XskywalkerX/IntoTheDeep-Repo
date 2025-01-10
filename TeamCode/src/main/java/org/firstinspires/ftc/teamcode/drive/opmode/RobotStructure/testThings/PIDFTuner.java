package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;

@TeleOp
@Config
public class PIDFTuner extends LinearOpMode {

    PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
    PIDFController pidfController = new PIDFController(pidCoefficients, kV, kA, kStatic);

    MProfile mp;

    volatile public static DcMotorEx linear;

    public static double TARGET = 0.7;

    volatile public static double kP = 0.05;
    volatile public static double kI = 0;
    volatile public static double kD = 0;

    public static double kV = 0.000111607;
    public static double kA = 0.0002;
    public static double kStatic = 0;

    public static double MAX_VEL = 8960.0;
    public static double MAX_ACCEL = 2000.0;

    volatile public static int tp = 5000;

    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {

        linear = hardwareMap.get(DcMotorEx.class, "linear");
        arm = hardwareMap.get(Servo.class, "arm");

        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        mp = new MProfile(pidCoefficients, pidfController, linear, MAX_VEL, MAX_ACCEL);

        Thread t1 = new Thread(mp);

        waitForStart();
        t1.start();
        while(opModeIsActive()) {

            pidCoefficients = new PIDCoefficients(kP, kI, kD);
            pidfController = new PIDFController(pidCoefficients, kV, kA, kStatic);

            arm.setPosition(TARGET);
            linear.setTargetPosition(tp);

            telemetry.addData("TARGET VEL", mp.getVelocity());
            telemetry.addData("CURRENT VEL", linear.getVelocity());
            telemetry.addData("CURRENT POSITION", linear.getCurrentPosition());
            telemetry.update();
        }
        t1.interrupt();
    }
}
