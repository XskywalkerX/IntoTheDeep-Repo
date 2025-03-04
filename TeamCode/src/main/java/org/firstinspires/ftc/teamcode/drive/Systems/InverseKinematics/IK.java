package org.firstinspires.ftc.teamcode.drive.Systems.InverseKinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IK {

    double L1, L2, SERVO_MIN, SERVO_MAX;
    Telemetry telemetry;

    public IK(double SERVO_MIN, double SERVO_MAX,
              double L1, double L2,
              Telemetry telemetry) {

        this.SERVO_MIN = SERVO_MIN;
        this.SERVO_MAX = SERVO_MAX;
        this.L1 = L1;
        this.L2 = L2;

        this.telemetry = telemetry;
    }

    public double[] inverseKinematics(double x_d, double y_d, double z_d) {
        // Compute theta1 (rotation in XY plane)
        double theta1 = Math.atan2(y_d, x_d);

        // Compute position of second joint
        double x1 = L1 * Math.cos(theta1);
        double y1 = L1 * Math.sin(theta1);

        // Compute vector from joint 2 to end-effector
        double dx = x_d - x1;
        double dy = y_d - y1;
        double dz = z_d;

        // Compute theta2 (rotation in XZ plane)
        double theta2 = Math.atan2(dz, Math.sqrt(dx * dx + dy * dy));

        // Check for singularities
        double distanceSq = dx * dx + dy * dy + dz * dz;
        if (Math.abs(distanceSq - (L1 + L2) * (L1 + L2)) < 1e-6 || Math.abs(distanceSq - (L1 - L2) * (L1 - L2)) < 1e-6) {
            telemetry.addData("Warning", "Singularity detected!");
            return null;
        }

        // Convert angles to degrees
        theta1 = Math.toDegrees(theta1);
        theta2 = Math.toDegrees(theta2);

        // Check joint limits
        if (theta1 < SERVO_MIN || theta1 > SERVO_MAX || theta2 < SERVO_MIN || theta2 > SERVO_MAX) {
            telemetry.addData("Warning", "Angles out of range!");
            return null;
        }

        return new double[]{theta1, theta2};
    }
}
