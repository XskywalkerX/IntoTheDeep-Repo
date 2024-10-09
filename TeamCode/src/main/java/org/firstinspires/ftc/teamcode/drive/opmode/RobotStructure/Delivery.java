package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Delivery {

    public static double MAX_ACCEL = 0;
    public static double MAX_VEL = 0;

    DcMotorEx linear;
    volatile public static boolean clipped = false;
    volatile public static boolean dunked = false;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    double currentPosition;
    double currentVelocity;

    PIDFController controller;
    PIDCoefficients coefficients;
    MProfile profileSegment;

    double startTime;
    double totalDistance;

    HardwareMap hwMap;

    public Delivery(HardwareMap hwMap) {

        this.hwMap = hwMap;

        linear = hwMap.get(DcMotorEx.class, "linear");
        coefficients = new PIDCoefficients(kp, ki, kd);
        controller = new PIDFController(
                coefficients,
                kV,
                kA,
                kStatic,
                (position, velocity) -> 0.0,
                NanoClock.system()
        );

        controller.setOutputBounds(-1.0, 1.0);

        totalDistance = linear.getTargetPosition();

        startTime = NanoClock.system().seconds();
    }

    public void update() {

        double currentTime = NanoClock.system().seconds() - startTime;

        profileSegment = getMProfileSegment(currentTime);

        controller.setTargetVelocity(profileSegment.velocity);
        controller.setTargetAcceleration(profileSegment.acceleration);

        currentPosition = linear.getCurrentPosition();
        currentVelocity = linear.getVelocity();

        double motorPower = controller.update(currentPosition, currentVelocity);

        linear.setPower(motorPower);
    }

    /**
     * Generates the current segment of the motion profile based on elapsed time.
     * This method computes the velocity and acceleration at each time step.
     */
    private MProfile getMProfileSegment(double elapsedTime) {
        // Time to reach max velocity under max acceleration
        double tAccel = MAX_VEL / MAX_ACCEL;

        // Distance covered during acceleration phase
        double dAccel = 0.5 * MAX_ACCEL * tAccel * tAccel;

        double velocity, acceleration;

        // Check the phase of the motion profile
        if (elapsedTime < tAccel) {
            // Acceleration phase
            acceleration = MAX_ACCEL;
            velocity = acceleration * elapsedTime;
        } else if (elapsedTime < (totalDistance / MAX_VEL)) {
            // Constant velocity (cruising phase)
            acceleration = 0;
            velocity = MAX_VEL;
        } else {
            // Deceleration phase
            double tDecel = elapsedTime - (totalDistance / MAX_VEL);
            acceleration = -MAX_ACCEL;
            velocity = MAX_VEL - (MAX_ACCEL * tDecel);
        }

        return new MProfile(velocity, acceleration);
    }

    /**
     * Class that holds the velocity and acceleration for each segment of the motion profile.
     */
    private static class MProfile {
        double velocity;
        double acceleration;

        MProfile(double velocity, double acceleration) {
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    // Telemetry for monitoring
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("LINEAR POWER", linear.getPower());
        telemetry.addData("LINEAR TARGET", linear.getTargetPosition());
        telemetry.addData("LINEAR POSITION", linear.getCurrentPosition());
        telemetry.update();
    }
}
