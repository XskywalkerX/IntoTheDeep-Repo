package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.DeliveryStates;
import org.firstinspires.ftc.teamcode.enums.IntakeStates;
import org.firstinspires.ftc.teamcode.enums.VerticalLinearStates;

public class DeliverySystem {

    DeliveryClaw deliveryClaw;
    VerticalLinear verticalLinear;

    double systemDelay = 1.5;

    public static DeliveryStates CS = DeliveryStates.INITIALIZE, PS = DeliveryStates.INITIALIZE;

    ElapsedTime time = new ElapsedTime();

    public static double kp = 0.02;
    PIDController linearPID = new PIDController(kp, 0, 0);

    public DeliverySystem() {
        CS = DeliveryStates.INITIALIZE;
        PS = DeliveryStates.INITIALIZE;

        deliveryClaw = new DeliveryClaw();
        verticalLinear = new VerticalLinear();
    }

    public void update(Robot robot, Telemetry telemetry, double manualPower) {
        switch (CS) {
            case INITIALIZE:
                VerticalLinear.CS = VerticalLinearStates.PREVENT_STATE;
                CS = DeliveryStates.TRANSFER;
            case TRANSFER:
                if (PS != CS) {
                    VerticalLinear.CS = VerticalLinearStates.PREVENT_STATE;
                    time.reset();
                }

                if (time.seconds() >= 2.5) {
                    VerticalLinear.CS = VerticalLinearStates.TRANSFER;
                }

                if (PS != DeliveryStates.TRANSFER && PS != DeliveryStates.INITIALIZE) {
                    time.reset();
                }
                DeliveryClaw.CS = ClawStates.OPENED;

                robot.deliveryX.setPosition(0);

                if (time.seconds() >= systemDelay) {
                    robot.deliveryY.setPosition(0.68);
                    time.reset();
                }
                break;
            case SPECIMEN_INTAKE:

                if (PS != DeliveryStates.SPECIMEN_INTAKE) {
                    time.reset();
                    DeliveryClaw.CS = ClawStates.OPENED;
                }

                robot.deliveryY.setPosition(0.4);
                if (time.seconds() >= systemDelay) {
                    robot.deliveryX.setPosition(0.93);
                    VerticalLinear.CS = VerticalLinearStates.RETRACTED;
                    time.reset();
                }
                break;
            case SPECIMEN_OUTTAKE_1:
                VerticalLinear.CS = VerticalLinearStates.CLIP_SPECIMEN;
                if (PS != DeliveryStates.SPECIMEN_OUTTAKE_1) {
                    time.reset();
                    IntakeClaw.CS = ClawStates.OPENED;
                }

                if (time.seconds() >= 0.5 && time.seconds() <= 0.9) {
                    DeliveryClaw.CS = ClawStates.CLOSED;
                }

                if (time.seconds() >= systemDelay) {
                    robot.deliveryX.setPosition(0.3);
                    time.reset();
                }
                robot.deliveryY.setPosition(0.1);
                break;
            case HIGH_BASKET:
                if (IntakeSystem.CS == IntakeStates.IDLE && ArmIntakeSystem.CS == ArmIntakeStates.TRANSFER && VerticalLinear.CS == VerticalLinearStates.TRANSFER) {
                    if (PS != DeliveryStates.HIGH_BASKET) {
                        time.reset();
                        IntakeClaw.CS = ClawStates.OPENED;
                    }
                    if (time.seconds() >= 0.5 && time.seconds() <= 0.9) {
                        DeliveryClaw.CS = ClawStates.CLOSED;
                    }

                    if (time.seconds() >= systemDelay) {
                        VerticalLinear.CS = VerticalLinearStates.HIGH_BASKET;
                        if (time.seconds() >= systemDelay) {
                            robot.deliveryX.setPosition(0.3);
                            time.reset();
                        }
                        robot.deliveryY.setPosition(0);
                    }
                } else {
                    IntakeSystem.CS = IntakeStates.IDLE;
                }
        }

        //robot.leftLinear.setPower(manualPower * 0.9);
        //robot.rightLinear.setPower(manualPower * 0.9);

        verticalLinear.update(robot);
        deliveryClaw.update(robot);


        PS = CS;

        telemetry.addData("Delivery Claw Position ", robot.deliveryClaw.getPosition());
        telemetry.addData("Delivery Claw State ", DeliveryClaw.CS.name());
        telemetry.addData("Delivery System timer ", time.seconds());
        telemetry.addData("Delivery System State ", CS.name());
        telemetry.addData("Delivery X Position ", robot.deliveryX.getPosition());
        telemetry.addData("Delivery Y Position ", robot.deliveryY.getPosition());
        telemetry.addData("Delivery Claw State ", DeliveryClaw.CS.name());

        telemetry.addData("Vertical State", VerticalLinear.CS.name());
        telemetry.addData("Left Vertical Linear Target Position", robot.leftLinear.getTargetPosition());
        telemetry.addData("Left Vertical Linear power", robot.leftLinear.getPower());
        telemetry.addData("Left Vertical Linear Current Position", robot.leftLinear.getCurrentPosition());

        telemetry.addData("Right Vertical Linear Target Position", robot.rightLinear.getTargetPosition());
        telemetry.addData("Right Vertical Linear power", robot.rightLinear.getPower());
        telemetry.addData("Right Vertical Linear Current Position", robot.rightLinear.getCurrentPosition());

    }

}
