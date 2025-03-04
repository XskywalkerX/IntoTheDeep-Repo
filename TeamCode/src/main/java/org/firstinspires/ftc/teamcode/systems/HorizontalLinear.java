package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.HorizontalLinearStates;

@Config
public class HorizontalLinear {

    public static double Kp_extend = 0.1;
    public static double Ki_extend = 0;
    public static double Kd_extend = 0.2;

    public static double Kp_retract = 0.1;
    public static double Ki_retract = 0;
    public static double Kd_retract = 0.2;

    public static HorizontalLinearStates CS = HorizontalLinearStates.INITIALIZE, PS = HorizontalLinearStates.INITIALIZE;

    ElapsedTime timer = new ElapsedTime();

    PIDController linearPIDExtend;
    PIDController linearPIDRetract;

    PIDController activePID;

    public HorizontalLinear() {
        CS = HorizontalLinearStates.INITIALIZE;
        PS = HorizontalLinearStates.INITIALIZE;

        linearPIDExtend = new PIDController(Kp_extend, Ki_extend, Kd_extend);
        linearPIDRetract = new PIDController(Kp_retract, Ki_retract, Kd_retract);

        activePID = linearPIDExtend;
    }

    public void update(Robot robot, double manualPower) {
        linearPIDExtend.setP(Kp_extend);
        linearPIDExtend.setI(Kp_extend);
        linearPIDExtend.setD(Kd_extend);

        linearPIDRetract.setP(Kp_retract);
        linearPIDRetract.setI(Ki_retract);
        linearPIDRetract.setD(Kd_retract);

        switch (CS) {
            case INITIALIZE:
            case RETRACTED:
                robot.horizontalLinear.setPower(-0.15);
                break;
            case EXTENDED:
                break;
            case EXTENDING:
                if (PS != HorizontalLinearStates.EXTENDING) {
                    robot.horizontalLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.horizontalLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                if (robot.horizontalLinear.getCurrentPosition() <= 250) {
                    robot.horizontalLinear.setPower(0.55);
                } else {
                    robot.horizontalLinear.setPower(0.35);
                }
                if (robot.horizontalLinear.getCurrentPosition() >= 320) {
                    CS = HorizontalLinearStates.EXTENDED;
                }
                break;
            case RETRACTING:
                if (PS != HorizontalLinearStates.RETRACTING) {
                    timer.reset();
                }
                if (timer.seconds() >= 2) {
                    CS = HorizontalLinearStates.RETRACTED;
                }
                robot.horizontalLinear.setPower(-0.55);
                break;
            case IDLE:
                //robot.horizontalLinear.setTargetPosition(robot.horizontalLinear.getCurrentPosition());
                break;
        }
        PS = CS;
    }
}
