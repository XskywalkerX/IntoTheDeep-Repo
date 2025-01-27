package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSystem {

    public static double MAX_POSITION = 8000;

    volatile public static double kP = 0.9;
    volatile public static double kI = 0;
    volatile public static double kD = 0.5;
    volatile public static double kF = 1.1;

    Telemetry telemetry;
    DcMotorEx expansion, reaper;

    Servo armLeft, armRight, box;

    Controller controller;

    public IntakeSystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        reaper = hwMap.get(DcMotorEx.class, "reaper");
        expansion = hwMap.get(DcMotorEx.class, "expansion");
        armLeft = hwMap.get(Servo.class, "armLeft");
        armRight = hwMap.get(Servo.class, "armRight");
        box = hwMap.get(Servo.class, "box");

        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //controller = new Controller(kP, kI, kD, kF, telemetry, expansion);
    }


    //TeleOp Functions
    public void moveExpansion(ElapsedTime time) {
        controller.moveMotor(time);
    }

    public void controlIntake() {

//        double error = tp - cp;
//
//        double target = error / tp;
//
//        moveArm(target);
//        moveBox(target);

        moveBox((double) expansion.getCurrentPosition() / MAX_POSITION);
        moveArm((double) expansion.getCurrentPosition() / MAX_POSITION);
    }

    /////////////////////////////////////////////////////////////////////////


    //minor movements
    public void moveArm(double tp) {
        armLeft.setPosition(tp);
        armRight.setPosition(1.0 - tp);
    }

    public void moveBox(double tp) {
        box.setPosition(tp);
    }

    public DcMotorEx getExpansion() {
        return expansion;
    }
}