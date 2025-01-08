package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;

@TeleOp
public class PIDFTuner extends LinearOpMode {

    PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
    PIDFController pidfController = new PIDFController(pidCoefficients, kV, kA, kStatic);

    MProfile mp;

    volatile public static DcMotorEx linear;

    volatile public static double kP = 0;
    volatile public static double kI = 0;
    volatile public static double kD = 0;

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 8960.0;
    public static double MAX_ACCEL = 2000.0;

    volatile public static int tp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        mp = new MProfile(pidCoefficients, pidfController, linear, MAX_VEL, MAX_ACCEL);

        Thread t1 = new Thread(mp);

        waitForStart();
        t1.start();
        while(opModeIsActive()) {
            linear.setTargetPosition(tp);

            telemetry.addData("TARGET VEL", mp.getVelocity());
            telemetry.addData("CURRENT VEL", linear.getVelocity());
            telemetry.addData("CURRENT POSITION", linear.getCurrentPosition());
            telemetry.update();

            sleep(150);
        }
        t1.interrupt();
    }
}
