package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MProfile implements Runnable {

    volatile public double MAX_VEL, MAX_ACCEL, cp, tp;

    volatile public ElapsedTime time = new ElapsedTime();

    volatile public double velocity = 0;
    volatile public double distance = 0;
    volatile public double acceleration = 0;

    PIDCoefficients coefficients;
    PIDFController controller;

    volatile public DcMotorEx motor;

    public MProfile(
            PIDCoefficients coefficients,
            PIDFController controller,
            DcMotorEx motor,
            double MAX_VEL,
            double MAX_ACCEL) {

        this.controller = controller;
        this.coefficients = coefficients;

        this.motor = motor;

        this.cp = motor.getCurrentPosition();
        this.tp = motor.getTargetPosition();

        this.MAX_VEL = MAX_VEL;
        this.MAX_ACCEL = MAX_ACCEL;
    }

    public MProfile(
            PIDCoefficients coefficients,
            PIDFController controller,
            DcMotorEx motor,
            double MAX_VEL,
            double MAX_ACCEL,
            double tp) {

        this.controller = controller;
        this.coefficients = coefficients;

        this.motor = motor;

        this.cp = motor.getCurrentPosition();
        this.tp = tp;

        this.MAX_VEL = MAX_VEL;
        this.MAX_ACCEL = MAX_ACCEL;
    }

    @Override
    public void run() {

        time.reset();

        while (!Thread.currentThread().isInterrupted()) {
            this.cp = motor.getCurrentPosition();
            this.tp = motor.getTargetPosition();
            System.out.println("CP IS " + this.cp + " | TP IS " + this.tp);
            System.out.println(velocity);

            if (cp < Math.abs(tp)) {
                velocity = rectFunction();
            } else {
                velocity = 0;
            }
            controller.setOutputBounds(0, MAX_VEL);
            controller.setTargetAcceleration(acceleration);
            controller.setTargetVelocity(velocity);
            controller.setTargetPosition(tp);

            motor.setVelocity(controller.update(cp, motor.getVelocity()));
        }
    }


    public double rectFunction() {

        double dAccel = 0.5 * MAX_VEL * MAX_VEL / MAX_ACCEL;
        double dCruise = tp - (dAccel + dAccel);

        double dRemaining = tp - cp;

        if (dRemaining > (dAccel + dCruise)) {
            System.out.println("ACCEL");
            acceleration = MAX_ACCEL;
            velocity = cp > 0 ? Math.sqrt(2 * MAX_ACCEL * cp) : Math.sqrt(2 * MAX_ACCEL * 0.1);
            distance = 0.5 * acceleration * cp * cp;
        } else if (dRemaining > dAccel) {
            System.out.println("CRUISE");
            acceleration = 0;
            velocity = MAX_VEL;
            distance = dAccel + MAX_VEL * (cp - dAccel);
        } else {
            System.out.println("DECEL");
            double dDecel = tp - cp;
            acceleration = -MAX_ACCEL;
            velocity = Math.sqrt(2 * MAX_ACCEL * dDecel);
            distance = tp - 0.5 * acceleration * (dRemaining * dRemaining);
        }

        return velocity;
    }


    public double expFunction(double time) {
        return time * time;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAccel() {
        return acceleration;
    }

    public double getTime() {
        return time.seconds();
    }

    public void resetTimer() {
        time.reset();
    }
}
