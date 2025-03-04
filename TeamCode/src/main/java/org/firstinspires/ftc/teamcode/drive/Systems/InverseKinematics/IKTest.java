package org.firstinspires.ftc.teamcode.drive.Systems.InverseKinematics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "InverseKinematicsOpMode")
@Disabled
public class IKTest extends LinearOpMode {

    public static double x_d = 8;
    public static double y_d = 5;
    public static double z_d = 10;

    // Constants
    private static final double L1 = 10.0; // Length of Link 1
    private static final double L2 = 10.0; // Length of Link 2
    private static final double SERVO_MIN = -135; // Minimum servo angle in degrees
    private static final double SERVO_MAX = 135;  // Maximum servo angle in degrees

    Servo servoA;
    Servo servoB;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize servos
        servoA = hardwareMap.get(Servo.class, "armX");
        servoB = hardwareMap.get(Servo.class, "armY");

        servoB.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Example target position (can be changed dynamically)

            double[] angles = inverseKinematics(x_d, y_d, z_d);
            if (angles != null) {
                double theta1 = angles[0];
                double theta2 = angles[1];

                // Normalize servo position (assuming 0.0 to 1.0 range)
                double servoPosA = (theta1 - SERVO_MIN) / (SERVO_MAX - SERVO_MIN);
                double servoPosB = (theta2 - SERVO_MIN) / (SERVO_MAX - SERVO_MIN);

                servoA.setPosition(servoPosA);
                servoB.setPosition(servoPosB);

                telemetry.addData("theta1 (Servo A)", "%.2f°", theta1);
                telemetry.addData("theta2 (Servo B)", "%.2f°", theta2);
                telemetry.addData("SERVO A", servoA.getPosition());
                telemetry.addData("SERVO B", servoB.getPosition());
            } else {
                telemetry.addData("Status", "No valid solution found.");
            }
            telemetry.update();
        }
    }

    private double[] inverseKinematics(double x_d, double y_d, double z_d) {
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