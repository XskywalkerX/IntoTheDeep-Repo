package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Controller {

    DcMotorEx motor;

    Telemetry telemetry;

    List<Integer> lastPos = new ArrayList<>();

    boolean hasUpdatedLastPosition = false;

    double velocity, acceleration, distance, integralSum, error, lastError = 0;

    double  kp, ki, kd, kf;

    int tp = 0;
    int THRESHOLD = 80;

    public Controller(
            double kp, double ki, double kd, double kf,
            Telemetry telemetry, DcMotorEx motor) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.telemetry = telemetry;
        this.motor = motor;
    }

    public void moveMotor(ElapsedTime time) {

        error = tp - motor.getCurrentPosition();

        double targetVelocity = rectFunction(1.0 / 3.0, 1.0 / 3.0,
                2664, 2600);

        motor.setVelocity(PIDFController(targetVelocity, motor.getVelocity(), time.seconds()));

        if (Math.abs(error) <= THRESHOLD && !hasUpdatedLastPosition) {
            lastPos.add(motor.getCurrentPosition());
            hasUpdatedLastPosition = true; // Prevent further updates for this movement
            telemetry.addLine("Updated lastPositions");
        }

        if (Math.abs(error) > THRESHOLD) {
            hasUpdatedLastPosition = false;
        }

        telemetry.addData("TARGET VEL", velocity);
        telemetry.addData("CURRENT VEL", motor.getVelocity());
        telemetry.addData("CURRENT POSITION", motor.getCurrentPosition());
        telemetry.addData("ERROR", error);
        telemetry.update();
    }

    public double rectFunction(double accelPeriod, double decelPeriod,
                               double MAX_VEL, double MAX_ACCEL) {

        double cp = motor.getCurrentPosition();

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

        double feedForward = reference * kf;

        lastError = error;

        return (error * kp) + (integralSum * ki) + (derivative * kd) + feedForward;
    }
}
