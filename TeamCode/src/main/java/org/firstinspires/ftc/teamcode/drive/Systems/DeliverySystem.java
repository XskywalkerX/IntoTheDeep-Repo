package org.firstinspires.ftc.teamcode.drive.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DeliverySystem {


    Telemetry telemetry;

    volatile public static double kP = 0.5;
    volatile public static double kI = 0;
    volatile public static double kD = 0;
    volatile public static double kF = 0.7;

    DcMotorEx linear;


    Controller controller;

    public DeliverySystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        linear = hwMap.get(DcMotorEx.class, "linear");
        //dBox = hwMap.get(Servo.class, "dBox");

        linear.setDirection(DcMotorSimple.Direction.REVERSE);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveLinear(ElapsedTime time) {
        controller.moveMotor(time);
    }

    public void updateLastPos(int lastPos) {
        controller.updateLastPos(lastPos);
    }

    public void moveBox(double tp) {
        //dBox.setPosition(tp);
    }

    public void openClaw() {
        //claw.setPosition(0.0);
    }

    public void closeClaw() {
        //claw.setPosition(1.0);
    }

    public void initSpecimen() {
        //armY.setPosition(Globals.ARM_Y_CLIP);
    }

    public void armRight() {
        closeClaw();
        //armX.setPosition(Globals.ARM_X_MAX);
    }

    public void arm_x(Gamepad gamepad){
        if (gamepad.dpad_left){
            //armX.setPosition(Globals.ARM_X_MAX);
        } else{
            //armX.setPosition(Globals.ARM_X_MIN);
        }
        //armX.setPosition(Globals.ARM_Y_CLIP);
        if (gamepad.x){
            openClaw();
        } else{
            closeClaw();
        }

    }

    public void armLeft() {
        //armX.setPosition(Globals.ARM_X_MIN);
    }

    public DcMotorEx getLinear() {
        return linear;
    }
}