package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotStructure.MProfile;

@Config
@TeleOp
@Disabled
public class newMProfileTest extends LinearOpMode {

    volatile public static int tp = 25000;

    public static double MAX_VEL = 2000;
    public static double MAX_ACCEL = 1500;

    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;

    public static double kV = 0.55;
    public static double kA = 0;
    public static double kStatic = 0;

    DcMotorEx motor;

    MProfile mp;

    PIDCoefficients coefficients;
    PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        coefficients = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(coefficients, kV, kA, kStatic);

        motor = hardwareMap.get(DcMotorEx.class, "mprofile");

        mp = new MProfile(coefficients, controller, motor, MAX_VEL, MAX_ACCEL);

        Thread t1 = new Thread(mp);

        waitForStart();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        t1.start();

        while(opModeIsActive()) {
            motor.setTargetPosition(tp);

            if (motor.getCurrentPosition() >= motor.getTargetPosition()) {
                sleep(2000);
                mp.resetTimer();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("CURRENT POS", motor.getCurrentPosition());
            telemetry.addData("TARGET POS", motor.getTargetPosition());
            telemetry.addData("CURRENT VELOCITY", motor.getVelocity());
            telemetry.addData("TARGET VELOCITY", mp.getVelocity());
            telemetry.addData("TARGET ACCEL", mp.getAccel());
            telemetry.update();
        }
        t1.interrupt();
    }
}