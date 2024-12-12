/*package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;

@TeleOp
@Config
public class test_linear extends LinearOpMode {

    double MAX_VEL = 14560.0 * 0.8;
    double MAX_ACCEL = 14560.0 * 0.8;

    ElapsedTime time = new ElapsedTime();

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

    public static double kV = 0.55;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double kP = 0.008;
    public static double kI = 0.0005;
    public static double kD = 0.0005;

    PIDCoefficients pidCoefficients;
    PIDFController controller;
    Servo servo;
    DcMotorEx linear;
    Mode mode = Mode.AUTO;
    Level level = Level.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        linear = hardwareMap.get(DcMotorEx.class, "linear");
        servo = hardwareMap.get(Servo.class, "servo");
        pidCoefficients = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(pidCoefficients, kV, kA, kStatic);
        //mp = new MProfile(pidCoefficients, controller, linear, MAX_VEL, MAX_ACCEL);

        Thread motionProfile = new Thread(mp);
        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        motionProfile.start();

        while (opModeIsActive()) {

            double tp = 0;

            if(gamepad1.right_bumper) {
                servo.setPosition(1);
            } else if(gamepad1.left_bumper) {
                servo.setPosition(0);
            }

            switch (mode) {

                case AUTO:

                    if (gamepad1.x) {
                        mode = Mode.MANUAL;
                    }

                    switch (level) {
                        case LOW:

                            if (time.seconds() > 0.2) {
                                linear.setTargetPosition(1000);

                                if (gamepad1.dpad_down) {
                                    time.reset();
                                    level = Level.IDLE;
                                } else if (gamepad1.dpad_up) {
                                    time.reset();
                                    level = Level.MID;
                                }
                            }

                            break;
                        case MID:

                            if (time.seconds() > 0.2) {
                                linear.setTargetPosition(1500);

                                if (gamepad1.dpad_down) {
                                    time.reset();
                                    level = Level.LOW;
                                } else if (gamepad1.dpad_up) {
                                    time.reset();
                                    level = Level.HIGH;
                                }
                            }

                            break;
                        case HIGH:

                            if (time.seconds() > 0.2) {
                                linear.setTargetPosition(2000);

                                if (gamepad1.dpad_down) {
                                    time.reset();
                                    level = Level.MID;
                                }
                            }

                            break;
                        case IDLE:

                            if (time.seconds() > 0.2) {
                                linear.setTargetPosition(0);
                                if (gamepad1.dpad_up) {
                                    time.reset();
                                    level = Level.LOW;
                                }
                            }

                            break;
                    }

                    break;
                case MANUAL:
                    motionProfile.interrupt();
                    tp += (gamepad1.right_trigger - gamepad1.left_trigger) * 100000;
                    //linear.setTargetPosition((int) tp);
                    linear.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    if (gamepad1.a) {
                        mode = Mode.AUTO;
                    }
                    break;
            }

            telemetry.addData("CURRENT POS", linear.getCurrentPosition());
            telemetry.addData("TARGET POS", linear.getTargetPosition());
            telemetry.update();
        }
        motionProfile.interrupt();
    }
}
*/