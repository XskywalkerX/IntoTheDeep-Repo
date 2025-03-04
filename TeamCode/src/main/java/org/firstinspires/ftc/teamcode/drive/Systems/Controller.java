package org.firstinspires.ftc.teamcode.drive.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

@Config
public class Controller {

    public static double accelTime = 1.0 / 3.0;

    double MAX_VEL, MAX_ACCEL;

    DcMotorEx motor;

    Telemetry telemetry;

    List<Integer> lastPos = new ArrayList<>();

    boolean hasUpdatedLastPosition = false;

    double acceleration, distance, integralSum, error, lastError = 0;
    volatile public static double velocity = 0;

    double kp, ki, kd, kf;

    int tp = 0;
    int THRESHOLD = 80;

    public Controller(
            double kp, double ki, double kd, double kf,
            Telemetry telemetry, DcMotorEx motor,
            double MAX_VEL, double MAX_ACCEL) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.telemetry = telemetry;
        this.motor = motor;
        this.MAX_VEL = MAX_VEL;
        this.MAX_ACCEL = MAX_ACCEL;
    }

    public void moveMotor(ElapsedTime time) {

        this.tp = motor.getTargetPosition();
        error = tp - motor.getCurrentPosition();

        double targetVelocity = rectFunction(time);

        motor.setVelocity(PIDFController(targetVelocity, motor.getVelocity(), time.seconds()));

        if (Math.abs(error) <= THRESHOLD && !hasUpdatedLastPosition) {
            lastPos.add(motor.getCurrentPosition());
            hasUpdatedLastPosition = true; // Prevent further updates for this movement
            telemetry.addLine("Updated lastPositions");
        }

        if (Math.abs(error) > THRESHOLD) {
            hasUpdatedLastPosition = false;
        }
    }

    public double rectFunction(ElapsedTime time) {

        double tAccel = MAX_VEL / MAX_ACCEL;
        double distance = !lastPos.isEmpty() ? tp - lastPos.get(lastPos.size() - 1) : tp;
        double halfway_distance = Math.abs(distance) / 2;
        double dAccel = 0.5 * MAX_ACCEL * Math.pow(tAccel, 2);

        if (dAccel > halfway_distance) {
            tAccel = Math.sqrt(halfway_distance / (0.5 * MAX_ACCEL));
            MAX_VEL = MAX_ACCEL * tAccel;
        }

        dAccel = 0.5 * MAX_ACCEL * Math.pow(tAccel, 2);
        double tDecel = tAccel;
        double dCruise = Math.abs(distance) - (2 * dAccel);
        double tCruise = dCruise / MAX_VEL;
        double tTotal = tAccel + tCruise + tDecel;

        double velocity = 0;

        if (Math.abs(distance) > THRESHOLD) {
            if (time.seconds() >= tTotal) {
                return distance;
            }
            if (time.seconds() < tAccel) {
                velocity = 0.5 * MAX_ACCEL * Math.pow(time.seconds(), 2);
            } else if (time.seconds() < tAccel + tCruise) {
                double etCruise = time.seconds() - tAccel;
                velocity = dAccel + (MAX_VEL * etCruise);
            } else {
                double etDecel = time.seconds() - (dAccel + dCruise);
                velocity = dAccel + dCruise + (MAX_VEL * etDecel) - (0.5 * MAX_ACCEL * Math.pow(etDecel, 2));
            }
        } else {
            velocity = 0;
        }

        return distance < 0 ? -velocity : velocity;
    }

    public double rectFunction(double accelPeriod, double decelPeriod) {

        double cp = motor.getCurrentPosition();

        double d = tp - lastPos.get(lastPos.size() - 1);

        double dAccel = accelPeriod * (d);
        double ddDecel = decelPeriod * (d);
        double dCruise = (Math.abs(d) - (Math.abs(dAccel) + Math.abs(ddDecel)));

        double dRemaining = tp - cp;

        if (Math.abs(dRemaining) > THRESHOLD) {
            if (d > 0) {
                if (dRemaining > (ddDecel + dCruise)) {
                    System.out.println("ACCEL");
                    telemetry.addLine("ACCEL" + motor.getDeviceName());
                    acceleration = MAX_ACCEL;
                    velocity = cp != 0 ? Math.sqrt(2 * MAX_ACCEL * Math.abs(cp)) : Math.sqrt(2 * MAX_ACCEL * 0.01);
                    //velocity = Math.sqrt(2 * MAX_ACCEL * (cp + 1));
                    distance = 0.5 * acceleration * cp * cp;
                } else if (dRemaining > ddDecel) {
                    System.out.println("CRUISE");
                    telemetry.addLine("CRUISE" + motor.getDeviceName());
                    acceleration = 0;
                    velocity = Math.sqrt(2 * MAX_ACCEL * dCruise);
                    distance = dAccel + MAX_VEL * (cp - dAccel);
                } else {
                    System.out.println("DECEL");
                    telemetry.addLine("DECEL" + motor.getDeviceName());
                    acceleration = -MAX_ACCEL;
                    velocity = Math.sqrt(2 * MAX_ACCEL * Math.abs(dRemaining));
                    distance = tp - 0.5 * acceleration * (dRemaining * dRemaining);
                }
            } else if (d < 0) {
                if (Math.abs(dRemaining) > (Math.abs(ddDecel) + dCruise)) {
                    telemetry.addLine("ACCEL" + motor.getDeviceName());
                    velocity = cp != 0 ? -Math.sqrt(2 * MAX_ACCEL * Math.abs(cp)) : -Math.sqrt(2 * MAX_ACCEL * 0.01);
                } else if (Math.abs(dRemaining) > Math.abs(ddDecel)) {
                    telemetry.addLine("CRUISE" + motor.getDeviceName());
                    velocity = -Math.sqrt(2 * MAX_ACCEL * Math.abs(dCruise));
                } else {
                    telemetry.addLine("DECEL" + motor.getDeviceName());
                    velocity = -Math.sqrt(2 * MAX_ACCEL * Math.abs(dRemaining));
                }
            }
        } else {
            velocity = 0.0;
        }
        telemetry.update();
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

    public void updateLastPos(int pos) {
        lastPos.add(pos);
    }

    public void clearPos() {
        lastPos.clear();
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public void setKf(double kf) {
        this.kf = kf;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public void setVELConstants(double MAX_VEL, double MAX_ACCEL) {
        this.MAX_VEL = MAX_VEL;
        this.MAX_ACCEL = MAX_ACCEL;
    }

    public void updateThreshold(int THRESHOLD) {
        this.THRESHOLD = THRESHOLD;
    }
}
