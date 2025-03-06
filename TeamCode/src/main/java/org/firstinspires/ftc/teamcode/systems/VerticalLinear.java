package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.VerticalLinearStates;

@Config
public class VerticalLinear {

    public static double Kp_retract = 0.00035;
    public static double Ki_retract = 0;
    public static double Kd_retract = 0.0;

    public static double Kp_extend = 0.007;
    public static double Ki_extend = 0;
    public static double Kd_extend = 0.000;

    public static int linearTransferPosition = 500;

    public static VerticalLinearStates CS = VerticalLinearStates.INITIALIZE, PS = VerticalLinearStates.INITIALIZE;

    PIDController linearPIDRetract;
    PIDController linearPIDExtend;

    PIDController activePID;

    public VerticalLinear() {
        CS = VerticalLinearStates.INITIALIZE;
        PS = VerticalLinearStates.INITIALIZE;

        linearPIDRetract = new PIDController(Kp_retract, Ki_retract, Kd_retract);
        linearPIDExtend = new PIDController(Kp_extend, Ki_extend, Kd_extend);

        activePID = linearPIDRetract;
    }

    public void update(Robot robot) {
        linearPIDRetract.setP(Kp_retract);
        linearPIDRetract.setI(Ki_retract);
        linearPIDRetract.setD(Kd_retract);

        linearPIDExtend.setP(Kp_extend);
        linearPIDExtend.setI(Ki_extend);
        linearPIDExtend.setD(Kd_extend);

        switch (CS) {
            case INITIALIZE:
            case RETRACTED:
                robot.leftLinear.setTargetPosition(0);
                robot.rightLinear.setTargetPosition(0);
                break;
            case CLIP_SPECIMEN:
                robot.leftLinear.setTargetPosition(750);
                robot.rightLinear.setTargetPosition(750);
                break;
            case TRANSFER:
                robot.leftLinear.setTargetPosition(linearTransferPosition);
                robot.rightLinear.setTargetPosition(linearTransferPosition);
                break;
            case HIGH_BASKET:
                robot.leftLinear.setTargetPosition(2190);
                robot.rightLinear.setTargetPosition(2190);
                break;
            case LOW_BASKET:
                robot.leftLinear.setTargetPosition(1905);
                robot.rightLinear.setTargetPosition(1905);
                break;
            case PREVENT_STATE:
                robot.leftLinear.setTargetPosition(linearTransferPosition + 350);
                robot.rightLinear.setTargetPosition(linearTransferPosition + 350);

        }

        if (robot.leftLinear.getCurrentPosition() <= robot.leftLinear.getTargetPosition()) {
            activePID = linearPIDExtend;
        } else {
            activePID = linearPIDRetract;
        }

        double motorsPower = -activePID.calculate(robot.rightLinear.getCurrentPosition(), robot.rightLinear.getTargetPosition());
        robot.leftLinear.setPower(motorsPower);
        robot.rightLinear.setPower(motorsPower);

        PS = CS;
    }
}
