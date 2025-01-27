package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class DeliverySystem {

    LinearLevel level = LinearLevel.GROUND;

    Telemetry telemetry;

    volatile public static double kP = 0.9;
    volatile public static double kI = 0;
    volatile public static double kD = 0.5;
    volatile public static double kF = 1.1;

    int tp = 0;

    DcMotorEx linear;

    Servo claw, armX, armY;

    Controller controller;

    public DeliverySystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        linear = hwMap.get(DcMotorEx.class, "linear");
        claw = hwMap.get(Servo.class, "claw");
        armX = hwMap.get(Servo.class, "armX");
        armY = hwMap.get(Servo.class, "armY");

        controller = new Controller(kP, kI, kD, kF, telemetry, linear);

        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveLinear(ElapsedTime time) {
        controller.moveMotor(time);
    }

    public void openClaw() {
        claw.setPosition(0.0);
    }

    public void closeClaw() {
        claw.setPosition(1.0);
    }

    public void armToClip() {
        closeClaw();
        armX.setPosition(1.0);
        armY.setPosition(0.6);
    }

    public DcMotorEx getLinear() {
        return linear;
    }
}