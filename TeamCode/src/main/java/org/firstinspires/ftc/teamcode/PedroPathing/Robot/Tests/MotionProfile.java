package org.firstinspires.ftc.teamcode.PedroPathing.Robot.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Systems.LinearLevel;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
public class MotionProfile {

    LinearLevel level = LinearLevel.GROUND;

    Gamepad gamepad2;

    ElapsedTime time;

    volatile public static double kP = 0.9;
    volatile public static double kI = 0;
    volatile public static double kD = 0.5;
    volatile public static double kF = 1.1;

    double integralSum = 0;
    double lastError = 0;

    double error = 0;

    boolean hasUpdatedLastPosition = false;

    List<Integer> lastPos = new ArrayList<>();

    int tp = 0;
    double velocity, distance, acceleration = 0;

    int THRESHOLD = 80;

    Telemetry telemetry;
    DcMotorEx linear, intake;

    public MotionProfile(
            HardwareMap hwMap,
            Telemetry telemetry,
            ElapsedTime time,
            Gamepad gamepad2) {

        this.time = time;
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
        linear = hwMap.get(DcMotorEx.class, "linear");

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void controlLinearTeleOp() {
        tp = linear.getTargetPosition();
        linear.setTargetPosition(level.getPosition());

        telemetry.addData("LEVEL", level);
        telemetry.update();

        switch (level) {
            case GROUND:

                if(gamepad2.dpad_up) {
                    level = LinearLevel.LOW_BASKET;
                }

                break;

            case LOW_BASKET:

                if(gamepad2.dpad_up) {
                    level = LinearLevel.CLIP;
                } else if (gamepad2.dpad_down) {
                    level = LinearLevel.GROUND;
                }
                break;

            case CLIP:

                if(gamepad2.dpad_up) {
                    level = LinearLevel.HIGH_BASKET;
                } else if (gamepad2.dpad_down) {
                    level = LinearLevel.LOW_BASKET;
                }
                break;

            case HIGH_BASKET:

                if(gamepad2.dpad_down) {
                    level = LinearLevel.CLIP;
                }

                break;
        }
    }


    public void moveLinear() {

        error = tp - linear.getCurrentPosition();

        double targetVelocity = rectFunction(1.0 / 3.0, 1.0 / 3.0,
                                             2664, 2600);

        linear.setVelocity(PIDFController(targetVelocity, linear.getVelocity(), time.seconds()));

        if (Math.abs(error) <= THRESHOLD && !hasUpdatedLastPosition) {
            lastPos.add(linear.getCurrentPosition());
            hasUpdatedLastPosition = true; // Prevent further updates for this movement
            telemetry.addLine("Updated lastPositions");
        }

        if (Math.abs(error) > THRESHOLD) {
            hasUpdatedLastPosition = false;
        }

        telemetry.addData("TARGET VEL", velocity);
        telemetry.addData("CURRENT VEL", linear.getVelocity());
        telemetry.addData("CURRENT POSITION", linear.getCurrentPosition());
        telemetry.addData("ERROR", error);
        telemetry.update();
    }


    public double rectFunction(double accelPeriod, double decelPeriod,
                               double MAX_VEL, double MAX_ACCEL) {

        double cp = linear.getCurrentPosition();

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

    public List<Integer> getLastPos() {
        return lastPos;
    }
}
