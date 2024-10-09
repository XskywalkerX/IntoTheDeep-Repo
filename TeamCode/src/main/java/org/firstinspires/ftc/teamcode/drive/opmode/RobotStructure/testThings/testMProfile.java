package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class testMProfile extends LinearOpMode {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double kV = 0.0035;
    public static double kA = 0.00002;
    public static double kStatic = 0;


    public static double MAX_VEL = 1500;
    public static double MAX_ACCEL = 1500;
    public static int tp = 0;

    double acceleration = 0;
    double vel = 0;
    double distance = 0;

    DcMotorEx mprofile;
    PIDFController controller;
    PIDCoefficients coefficients;

    @Override
    public void runOpMode() throws InterruptedException {

        coefficients = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(coefficients, kV, kA, kStatic);

        controller.setOutputBounds(-1, 1);

        mprofile = hardwareMap.get(DcMotorEx.class, "mprofile");
        mprofile.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        mprofile.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime pathDuration = new ElapsedTime();
        double previousVelocity = 0;
        double lastTime = pathDuration.seconds();  // Track previous loop time

        while (opModeIsActive()) {

            double tAccel = MAX_VEL / MAX_ACCEL;
            double dAccel = 0.5 * MAX_ACCEL * tAccel * tAccel;
            double dDecel = dAccel;
            double dCruise = tp - (dAccel + dDecel);
            double tCruise = dCruise > 0 ? dCruise / MAX_VEL : 0;
            double tTime = (2 * tAccel) + tCruise;

            double currentTime = pathDuration.seconds();  // Get current loop time
            double dt = currentTime - lastTime;  // Time step (dt)
            lastTime = currentTime;  // Update lastTime for the next iteration

            double newVelocity = mprofile.getVelocity();  // Get the current velocity

            // Calculate acceleration as change in velocity over time
            double accel = (newVelocity - previousVelocity);
            previousVelocity = newVelocity;  // Update previous velocity for the next iteration

            // Acceleration phase
            if (pathDuration.seconds() < tAccel) {
                acceleration = MAX_ACCEL;
                vel = acceleration * pathDuration.seconds();
                distance = 0.5 * acceleration * Math.pow(pathDuration.seconds(), 2);
            } else if (pathDuration.seconds() <= tCruise + tAccel) {  // Cruising phase
                acceleration = 0;
                vel = MAX_VEL;
                distance = dAccel + MAX_VEL * (pathDuration.seconds() - tAccel);
            } else {  // Deceleration phase
                double tDecel = pathDuration.seconds() - (tAccel + tCruise);
                acceleration = -MAX_ACCEL;
                vel = MAX_VEL - (MAX_ACCEL * tDecel);
                distance = dAccel + dCruise + (MAX_VEL * tDecel) - (0.5 * MAX_ACCEL * Math.pow(tDecel, 2));
            }

            // If time has passed the total time for the motion profile, reset everything
            if(pathDuration.seconds() >= tTime) {
                vel = 0;
                mprofile.setVelocity(0);
                mprofile.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(2000);
                pathDuration.reset();
                mprofile.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            controller.setTargetVelocity(vel);
            controller.setTargetAcceleration(acceleration);
            controller.setTargetPosition(controller.getTargetPosition());

            mprofile.setPower(controller.update(mprofile.getCurrentPosition(), mprofile.getVelocity()));

            // Telemetry to monitor current and target values
            telemetry.addData("CURRENT VEL", newVelocity);
            telemetry.addData("CURRENT POS", mprofile.getCurrentPosition());
            telemetry.addData("TARGET POS", tp);
            telemetry.addData("TARGET VEL", vel);
            telemetry.addData("TARGET ACCEL", acceleration);
            telemetry.addData("CURRENT ACCEL", accel);  // Display calculated accel
            telemetry.addData("PATH DURATION", pathDuration.seconds());
            telemetry.addData("PREVIOUS VEL", previousVelocity);
            telemetry.addData("NEW VELOCITY", newVelocity);
            telemetry.addData("dT", dt);
            telemetry.update();
        }
    }
}
