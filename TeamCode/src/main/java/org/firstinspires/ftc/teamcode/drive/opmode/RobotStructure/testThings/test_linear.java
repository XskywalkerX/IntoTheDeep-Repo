package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;

@TeleOp
public class test_linear extends LinearOpMode {

    double MAX_VEL = 14560.0 * 0.8;
    double MAX_ACCEL = 14560.0 * 0.8;

    MProfile mp;

    enum Mode {
        AUTO,
        MANUAL
    }

    enum Level {
        LOW,
        MID,
        HIGH,
        IDLE
    }

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    PIDCoefficients pidCoefficients;
    PIDFController controller;

    DcMotorEx linear;
    Mode mode = Mode.MANUAL;
    Level level = Level.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        linear = hardwareMap.get(DcMotorEx.class, "motor_expansion");
        pidCoefficients = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(pidCoefficients, kV, kA, kStatic);
        mp = new MProfile(pidCoefficients, controller, linear, MAX_VEL, MAX_ACCEL);

        Thread motionProfile = new Thread(mp);
        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        motionProfile.start();

        while(opModeIsActive()) {

            double tp = 0;

            switch(mode) {
                case AUTO:

                    switch(level) {
                        case LOW:

                            linear.setTargetPosition(1000);

                            if(gamepad1.left_stick_x < 0) {
                                level = Level.IDLE;
                            } else if (gamepad1.left_stick_x > 0) {
                                level = Level.MID;
                            }

                            break;
                        case MID:

                            linear.setTargetPosition(2500);

                            if(gamepad1.left_stick_x < 0) {
                                level = Level.LOW;
                            } else if (gamepad1.left_stick_x > 0) {
                                level = Level.HIGH;
                            }

                            break;
                        case HIGH:

                            linear.setTargetPosition(5000);

                            if(gamepad1.left_stick_x < 0) {
                                level = Level.MID;
                            }

                            break;
                        case IDLE:

                            linear.setTargetPosition(0);
                            if(gamepad1.left_stick_x > 0) {
                                level = Level.LOW;
                            }

                            break;
                    }

                    break;
                case MANUAL:
                    tp += (gamepad1.right_trigger - gamepad1.left_trigger) * 10;
                    linear.setTargetPosition((int) tp);
                    if(gamepad1.a) {
                        mode = Mode.AUTO;
                    }
                    break;
            }

            telemetry.addData("CURRENT POS", linear.getCurrentPosition());
            telemetry.addData("TARGET POS", linear.getTargetPosition());
        }
        motionProfile.interrupt();
    }
}
