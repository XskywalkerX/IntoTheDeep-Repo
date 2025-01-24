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

public class IntakeSystem {

    List<Integer> lastPos = new ArrayList<>();

    int tp = 0;

    double velocity, distance, acceleration = 0;

    double integralSum = 0;
    double lastError = 0;


    volatile public static double kP = 0.9;
    volatile public static double kI = 0;
    volatile public static double kD = 0.5;
    volatile public static double kF = 1.1;

    boolean hasUpdatedLastPosition = false;
    double error;

    int THRESHOLD = 80;

    ExpansionLevel level = ExpansionLevel.IDLE;

    Telemetry telemetry;
    DcMotorEx expansion, reaper;

    Servo armLeft, armRight, box;

    public IntakeSystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        expansion = hwMap.get(DcMotorEx.class, "expansion");
        armLeft = hwMap.get(Servo.class, "armLeft");
        armRight = hwMap.get(Servo.class, "armRight");
        box = hwMap.get(Servo.class, "box");

        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //TeleOp Functions
    public void controlIntake(Gamepad gamepad, ElapsedTime time) {
        expansion.setTargetPosition(level.getPosition());
        double error = expansion.getTargetPosition() - expansion.getCurrentPosition();

        double tp = error / expansion.getTargetPosition();
        moveExpansion(time);
        moveArm(tp);
        moveBox(tp);
        runReaper(gamepad);

        switch (level) {
            case IDLE:

                if(!expansion.isBusy()) {
                    if(gamepad.b) {
                        level = ExpansionLevel.BACK;
                    }
                    if(gamepad.x) {
                        level = ExpansionLevel.MIDDLE;
                    }
                    if(gamepad.y) {
                        level = ExpansionLevel.FRONT;
                    }
                }

                break;

            case BACK:

                if(!expansion.isBusy()) {
                    if(gamepad.b) {
                        level = ExpansionLevel.IDLE;
                    }
                    if(gamepad.x) {
                        level = ExpansionLevel.MIDDLE;
                    }
                    if(gamepad.y) {
                        level = ExpansionLevel.FRONT;
                    }
                }
                break;

            case MIDDLE:

                if(!expansion.isBusy()) {
                    if(gamepad.b) {
                        level = ExpansionLevel.IDLE;
                    }
                    if(gamepad.x) {
                        level = ExpansionLevel.BACK;
                    }
                    if(gamepad.y) {
                        level = ExpansionLevel.FRONT;
                    }
                }

                break;

            case FRONT:

                if(!expansion.isBusy()) {
                    if(gamepad.b) {
                        level = ExpansionLevel.IDLE;
                    }
                    if(gamepad.x) {
                        level = ExpansionLevel.BACK;
                    }
                    if(gamepad.y) {
                        level = ExpansionLevel.MIDDLE;
                    }
                }

                break;
        }
    }

    /////////////////////////////////////////////////////////////////////////

    public void moveExpansion(ElapsedTime time) {

        error = tp - expansion.getCurrentPosition();

        double targetVelocity = rectFunction(1.0 / 3.0, 1.0 / 3.0,
                2664, 2600);

        expansion.setVelocity(PIDFController(targetVelocity, expansion.getVelocity(), time.seconds()));

        if (Math.abs(error) <= THRESHOLD && !hasUpdatedLastPosition) {
            lastPos.add(expansion.getCurrentPosition());
            hasUpdatedLastPosition = true; // Prevent further updates for this movement
            telemetry.addLine("Updated lastPositions");
        }

        if (Math.abs(error) > THRESHOLD) {
            hasUpdatedLastPosition = false;
        }

        telemetry.addData("TARGET VEL", velocity);
        telemetry.addData("CURRENT VEL", expansion.getVelocity());
        telemetry.addData("CURRENT POSITION", expansion.getCurrentPosition());
        telemetry.addData("ERROR", error);
        telemetry.update();
    }

    public double rectFunction(double accelPeriod, double decelPeriod,
                               double MAX_VEL, double MAX_ACCEL) {

        double cp = expansion.getCurrentPosition();

        double d = tp - lastPos.get(lastPos.size() - 1);

        double dAccel = accelPeriod * (d);
        double ddDecel = decelPeriod * (d);
        double dCruise = (Math.abs(d) - (Math.abs(dAccel) + Math.abs(ddDecel)));

        double dRemaining = tp - cp;

        telemetry.addData("dRemaining", dRemaining);
        telemetry.addData("dAccel", dAccel);
        telemetry.addData("dCruise", dCruise);
        telemetry.addData("ddDecel", ddDecel);
        telemetry.addData("LAST POS", lastPos.get(lastPos.size() - 1));
        telemetry.update();

        if (Math.abs(dRemaining) > THRESHOLD) {
            if (d > 0) {
                if (dRemaining > (ddDecel + dCruise)) {
                    System.out.println("ACCEL");
                    telemetry.addLine("ACCEL");
                    acceleration = MAX_ACCEL;
                    velocity = cp != 0 ? Math.sqrt(2 * MAX_ACCEL * Math.abs(cp)) : Math.sqrt(2 * MAX_ACCEL * 0.01);
                    //velocity = Math.sqrt(2 * MAX_ACCEL * (cp + 1));
                    distance = 0.5 * acceleration * cp * cp;
                } else if (dRemaining > ddDecel) {
                    System.out.println("CRUISE");
                    telemetry.addLine("CRUISE");
                    acceleration = 0;
                    velocity = Math.sqrt(2 * MAX_ACCEL * dCruise);
                    distance = dAccel + MAX_VEL * (cp - dAccel);
                } else {
                    System.out.println("DECEL");
                    telemetry.addLine("DECEL");
                    acceleration = -MAX_ACCEL;
                    velocity = Math.sqrt(2 * MAX_ACCEL * Math.abs(dRemaining));
                    distance = tp - 0.5 * acceleration * (dRemaining * dRemaining);
                }
            } else if (d < 0) {
                if(Math.abs(dRemaining) > (Math.abs(ddDecel) + dCruise)) {
                    telemetry.addLine("ACCEL");
                    velocity = cp != 0 ? -Math.sqrt(2 * MAX_ACCEL * Math.abs(cp)) : -Math.sqrt(2 * MAX_ACCEL * 0.01);
                } else if (Math.abs(dRemaining) > Math.abs(ddDecel)){
                    telemetry.addLine("CRUISE");
                    velocity = -Math.sqrt(2 * MAX_ACCEL * Math.abs(dCruise));
                } else {
                    telemetry.addLine("DECEL");
                    velocity = - Math.sqrt(2 * MAX_ACCEL * Math.abs(dRemaining));
                }
            }
        } else {
            velocity = 0.0;
        }
        return velocity;
    }

    double PIDFController(double reference, double state, double dt) {

        double error = reference - state;

        integralSum += error * dt;

        double derivative = (error - lastError) / dt;

        double feedForward = reference * kF;

        lastError = error;

        return (error * kP) + (integralSum * kI) + (derivative * kD) + feedForward;
    }


    //minor movements
    public void moveArm(double tp) {
        armLeft.setPosition(tp);
        armRight.setPosition(1.0 - tp);
    }

    public void moveBox(double tp) {
        box.setPosition(tp);
    }

    public void runReaper(Gamepad gamepad) {
        reaper.setPower(gamepad.right_trigger - gamepad.left_trigger);
    }
}