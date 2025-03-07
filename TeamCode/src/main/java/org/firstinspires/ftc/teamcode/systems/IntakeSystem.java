package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.DeliveryStates;
import org.firstinspires.ftc.teamcode.enums.HorizontalLinearStates;
import org.firstinspires.ftc.teamcode.enums.IntakeStates;
import org.firstinspires.ftc.teamcode.enums.VerticalLinearStates;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@Config
public class IntakeSystem {

    boolean sampleAligned = false;
    public static double MULTIPLIER = 1;

    public static double k = 0.2;
    public static IntakeStates CS, PS = IntakeStates.INITIALIZE;

    HorizontalLinear horizontalLinear;
    IntakeClaw intakeClaw;
    ArmIntakeSystem armIntakeSystem;
    VerticalLinear verticalLinear;

    ElapsedTime alignTime = new ElapsedTime();
    ElapsedTime time = new ElapsedTime();
    PIDController intakeXPID;
    PIDController linearPID;

    public static double Kp_itkX = -0.00005;
    public static double Ki_itkX = 0.0001;
    public static double Kd_itkX = 0.000001;

    public static double linear_max_vel = 0.2;
    public static double Kp_linear = 0.0015;
    public static double Ki_linear = 0.01;
    public static double Kd_linear = 0.00001;

    public static double theta = 0;
    public static double theta2 = 0;

    List<ColorBlobLocatorProcessor.Blob> blobs;

    public IntakeSystem(List<ColorBlobLocatorProcessor.Blob> blobs) {

        this.blobs = blobs;

        CS = IntakeStates.INITIALIZE;
        PS = IntakeStates.INITIALIZE;

        horizontalLinear = new HorizontalLinear();
        intakeClaw = new IntakeClaw();
        armIntakeSystem = new ArmIntakeSystem();
        verticalLinear = new VerticalLinear();

        intakeXPID = new PIDController(Kp_itkX, Ki_itkX, Kd_itkX);
        linearPID = new PIDController(Kp_linear, Ki_linear, Kd_linear);
    }

    public void setBlobs(List<ColorBlobLocatorProcessor.Blob> blobs) {
        this.blobs = blobs;
    }

    public void update(Robot robot, double manualPower) {
        switch (CS) {
            case INITIALIZE:
            case IDLE:
                if (DeliverySystem.CS == DeliveryStates.TRANSFER) {
                    if (PS != CS) {
                        time.reset();
                    }
                    robot.clawB.setPosition(0);
                    VerticalLinear.CS = VerticalLinearStates.PREVENT_STATE;
                    if (time.seconds() >= 0.5) {
                        IntakeClaw.CS = ClawStates.CLOSED;
                        HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                        ArmIntakeSystem.CS = ArmIntakeStates.TRANSFER;
                    }
                    if (time.seconds() >= 3.3) {
                        VerticalLinear.CS = VerticalLinearStates.TRANSFER;
                    }
                } else {
                    if (PS != CS) {
                        IntakeClaw.CS = ClawStates.CLOSED;
                    }

                    HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                    ArmIntakeSystem.CS = ArmIntakeStates.TRANSFER;
                }
                break;
            case READING:

                MULTIPLIER *= 1 + (manualPower * k);

                if (PS != CS) {
                    alignTime.reset();
                    time.reset();
                    IntakeClaw.CS = ClawStates.OPENED;
                    ArmIntakeSystem.CS = ArmIntakeStates.READ;
                    robot.clawB.setPosition(0);
                }

                if (DeliverySystem.CS == DeliveryStates.TRANSFER) {
                    VerticalLinear.CS = VerticalLinearStates.PREVENT_STATE;

                    if (!blobs.isEmpty() && time.seconds() >= 0.5) {
                        Moments mu = Imgproc.moments(blobs.get(0).getContour());

                        // Compute orientation
                        double aa = mu.mu20 / mu.m00;
                        double bb = mu.mu11 / mu.m00;
                        double cc = mu.mu02 / mu.m00;
                        theta2 = 0.5 * Math.atan2(2 * bb, aa - cc) * 180 / Math.PI;
                        theta = Math.abs(theta2);


                        double servoOutputPosition = robot.intakeX.getPosition()
                                + intakeXPID.calculate(blobs.get(0).getBoxFit().center.x, Globals.CAMERA_X_CATCH_SETPOINT);

                        if (servoOutputPosition < 0) {
                            servoOutputPosition = 0;
                        } else if (servoOutputPosition > 1) {
                            servoOutputPosition = 1;
                        }

                        if (robot.intakeX.getPosition() < Globals.INTAKE_X_SAFE_MIN) {
                            robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MIN);
                        } else if (robot.intakeX.getPosition() > Globals.INTAKE_X_SAFE_MAX) {
                            robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MAX);
                        } else {
                            robot.intakeX.setPosition(servoOutputPosition);
                        }


                        double power = linearPID.calculate(blobs.get(0).getBoxFit().center.y, Globals.CAMERA_Y_CATCH_SETPOINT);

                        if (power > linear_max_vel) {
                            power = linear_max_vel;
                        } else if (power < -linear_max_vel) {
                            power = -linear_max_vel;
                        }

                        robot.horizontalLinear.setPower(power * MULTIPLIER);

                        if ((Math.abs(Globals.CAMERA_X_CATCH_SETPOINT - blobs.get(0).getBoxFit().center.x) < 15) && (Math.abs(Globals.CAMERA_Y_CATCH_SETPOINT - blobs.get(0).getBoxFit().center.y) < 55)) {

                            robot.clawB.setPosition((theta2 + 90) / 180);
                            sampleAligned = true;
                        } else {
                            alignTime.reset();
                        }

                        if(alignTime.seconds() >= 0.819 && sampleAligned) {
                            alignTime.reset();
                            CS = IntakeStates.CATCH;
                            ArmIntakeSystem.CS = ArmIntakeStates.CATCH;
                            sampleAligned = false;
                        }

                    } else {
                        robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                        robot.intakeY.setPosition(Globals.INTAKE_Y_READ);

                        if (HorizontalLinear.CS == HorizontalLinearStates.RETRACTED) {
                            HorizontalLinear.CS = HorizontalLinearStates.EXTENDING;
                        } else if (HorizontalLinear.CS == HorizontalLinearStates.EXTENDED) {
                            HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                        }
                    }

                } else {
                    if (!blobs.isEmpty()) {
                        Moments mu = Imgproc.moments(blobs.get(0).getContour());

                        // Compute orientation
                        double aa = mu.mu20 / mu.m00;
                        double bb = mu.mu11 / mu.m00;
                        double cc = mu.mu02 / mu.m00;
                        theta2 = Math.atan2(2 * bb, aa - cc) * 180 / Math.PI;
                        theta = Math.abs(theta2);


                        double servoOutputPosition = robot.intakeX.getPosition()
                                + intakeXPID.calculate(blobs.get(0).getBoxFit().center.x, Globals.CAMERA_X_CATCH_SETPOINT);

                        if (servoOutputPosition < 0) {
                            servoOutputPosition = 0;
                        } else if (servoOutputPosition > 1) {
                            servoOutputPosition = 1;
                        }

                        if (robot.intakeX.getPosition() < Globals.INTAKE_X_SAFE_MIN) {
                            robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MIN);
                        } else if (robot.intakeX.getPosition() > Globals.INTAKE_X_SAFE_MAX) {
                            robot.intakeX.setPosition(Globals.INTAKE_X_SAFE_MAX);
                        } else {
                            robot.intakeX.setPosition(servoOutputPosition);
                        }


                        double power = linearPID.calculate(blobs.get(0).getBoxFit().center.y, Globals.CAMERA_Y_CATCH_SETPOINT);

                        if (power > linear_max_vel) {
                            power = linear_max_vel;
                        } else if (power < -linear_max_vel) {
                            power = -linear_max_vel;
                        }

                        robot.horizontalLinear.setPower(power * MULTIPLIER);

                        if ((Math.abs(Globals.CAMERA_X_CATCH_SETPOINT - blobs.get(0).getBoxFit().center.x) < 15) && (Math.abs(Globals.CAMERA_Y_CATCH_SETPOINT - blobs.get(0).getBoxFit().center.y) < 55)) {

                            robot.clawB.setPosition((theta2 + 90) / 180);
                            sampleAligned = true;
                        } else {
                            alignTime.reset();
                        }
                        if (sampleAligned && alignTime.seconds() >= 0.819) {
                            alignTime.reset();
                            CS = IntakeStates.CATCH;
                            ArmIntakeSystem.CS = ArmIntakeStates.CATCH;
                            sampleAligned = false;
                        }

                    } else {
                        robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                        robot.intakeY.setPosition(Globals.INTAKE_Y_READ);

                        if (HorizontalLinear.CS == HorizontalLinearStates.RETRACTED) {
                            HorizontalLinear.CS = HorizontalLinearStates.EXTENDING;
                        } else if (HorizontalLinear.CS == HorizontalLinearStates.EXTENDED) {
                            HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                        }
                    }
                }
                break;

            case CATCH:
                if (PS != CS) {
                    ArmIntakeSystem.CS = ArmIntakeStates.CATCH;
                }
                break;
            case DROP:
                if (PS != CS) {
                    HorizontalLinear.CS = HorizontalLinearStates.EXTENDING;
                    VerticalLinear.CS = VerticalLinearStates.PREVENT_STATE;
                    robot.clawB.setPosition(0);
                    ArmIntakeSystem.CS = ArmIntakeStates.DROP;
                }
                break;
        }
        PS = CS;

        intakeXPID.setP(Kp_itkX);
        intakeXPID.setI(Ki_itkX);
        intakeXPID.setD(Kd_itkX);

        linearPID.setP(Kp_linear);
        linearPID.setI(Ki_linear);
        linearPID.setD(Kd_linear);

        //horizontalLinear.update(robot, manualPower);
        armIntakeSystem.update(robot);
        intakeClaw.update(robot);
        verticalLinear.update(robot);
    }
}