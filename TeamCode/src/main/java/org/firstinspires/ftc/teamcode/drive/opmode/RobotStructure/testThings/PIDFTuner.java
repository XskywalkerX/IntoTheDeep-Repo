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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class PIDFTuner extends LinearOpMode {

    double integralSum = 0;
    double lastError = 0;

    List<Integer> lastPositions = new ArrayList<>();

    volatile public static DcMotorEx linear;


    public static double TARGET = 0.7;

    volatile public static double kP = 0.9;
    volatile public static double kI = 0;
    volatile public static double kD = 0.5;
    volatile public static double kF = 1.1;

    public static double MAX_VEL = 2664;
    public static double MAX_ACCEL = 2000;

    volatile public static int tp = 5000;

    volatile public double acceleration = 0;
    volatile public double velocity = 0;
    volatile public double distance = 0;
    volatile public double error = 0;

    volatile public static double THRESHOLD = 10.0;
    boolean hasUpdatedLastPosition = false;

    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        linear = hardwareMap.get(DcMotorEx.class, "linear");
        arm = hardwareMap.get(Servo.class, "arm");

        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStarted()) {
            telemetry.addData("TARGET VEL", velocity);
            telemetry.addData("CURRENT VEL", linear.getVelocity());
            telemetry.addData("CURRENT POSITION", linear.getCurrentPosition());
            telemetry.addData("ERROR", error);
            telemetry.update();
        }

        waitForStart();

        lastPositions.add(linear.getCurrentPosition());

        while (opModeIsActive()) {

            error = tp - linear.getCurrentPosition();

            arm.setPosition(TARGET);

            double targetVelocity = rectFunction(1.0 / 3.0, 1.0 / 3.0);

            linear.setVelocity(PIDFController(targetVelocity, linear.getVelocity(), time.seconds()));

            if (Math.abs(error) <= THRESHOLD && !hasUpdatedLastPosition) {
                lastPositions.add(linear.getCurrentPosition());
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
    }

    public double rectFunction(double accelPeriod, double decelPeriod) {

        double cp = linear.getCurrentPosition();

        double d = tp - lastPositions.get(lastPositions.size() - 1);

        double dAccel = accelPeriod * (d);
        double ddDecel = decelPeriod * (d);
        double dCruise = (Math.abs(d) - (Math.abs(dAccel) + Math.abs(ddDecel)));

        double dRemaining = tp - cp;

        telemetry.addData("dRemaining", dRemaining);
        telemetry.addData("dAccel", dAccel);
        telemetry.addData("dCruise", dCruise);
        telemetry.addData("ddDecel", ddDecel);
        telemetry.addData("LAST POS", lastPositions.get(lastPositions.size() - 1));
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
                    double dDecel = tp - cp;
                    acceleration = -MAX_ACCEL;
                    velocity = Math.sqrt(2 * MAX_ACCEL * Math.abs(error));
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
                    velocity = -Math.sqrt(2 * MAX_ACCEL * Math.abs(error));
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
}
