package org.firstinspires.ftc.teamcode.PedroPathing.Robot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Systems.Controller;

@Config
@TeleOp(name = "PIDF Tuner", group = "SYSTEMS TUNING")
@Disabled
public class PIDFTuner extends LinearOpMode {

    Controller linearController;
    Controller expansionController;

    DcMotorEx linear;
    DcMotorEx reaper;
    DcMotorEx expansion;

    public static double cp = 0;

    volatile public static double kI, kD, power = 0;
    volatile public static double kP = 0.5;
    volatile public static double kF = 0.7;
    volatile public static double kI2, kD2, kF2 = 0;
    volatile public static double kP2 = 0.8;

    public static int tp = 0;
    public static int tp2 = 0;

    Servo claw, armX, armY;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        expansion = hardwareMap.get(DcMotorEx.class, "expansion");
        linear = hardwareMap.get(DcMotorEx.class, "linear");
        reaper = hardwareMap.get(DcMotorEx.class, "reaper");
        armX = hardwareMap.get(Servo.class, "armX");
        armY = hardwareMap.get(Servo.class, "armY");
        claw = hardwareMap.get(Servo.class, "claw");

        expansion.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        linearController.updateLastPos(linear.getCurrentPosition());
        expansionController.updateLastPos(expansion.getCurrentPosition());

        waitForStart();

        while(opModeIsActive()) {

            claw.setPosition(cp);
            armX.setPosition(0.2);
            armY.setPosition(0.77);


            telemetry.addData("LINEAR POS", linear.getCurrentPosition());
            telemetry.addData("LINEAR VEL", linear.getVelocity());
            telemetry.addData("INTAKE POS", expansion.getCurrentPosition());
            telemetry.addData("INTAKE VEL", expansion.getVelocity());


            linear.setTargetPosition(tp);
            linearController.moveMotor(time);

            expansion.setTargetPosition(tp2);
            expansionController.moveMotor(time);

            reaper.setPower(power);

            telemetry.update();
        }
        linearController.clearPos();
        expansionController.clearPos();
    }
}
