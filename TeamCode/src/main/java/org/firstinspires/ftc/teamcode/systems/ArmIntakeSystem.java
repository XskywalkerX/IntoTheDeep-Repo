package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.HorizontalLinearStates;
import org.firstinspires.ftc.teamcode.enums.IntakeStates;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
public class ArmIntakeSystem {

    public static ArmIntakeStates CS = ArmIntakeStates.INITIALIZE,
            PS = ArmIntakeStates.INITIALIZE;

    ElapsedTime time = new ElapsedTime();

    double clawBAngle = 0;

    HorizontalLinear horizontalLinear;

    public ArmIntakeSystem() {
        CS = ArmIntakeStates.INITIALIZE;
        PS = ArmIntakeStates.INITIALIZE;

        horizontalLinear = new HorizontalLinear();
    }

    public void updateClawAngle(double angle) {
        clawBAngle = (angle) / 180;
    }

    public void updateClawPosition(double angle) {
        clawBAngle = angle;
    }

    public void update(Robot robot, Telemetry telemetry) {
        switch (CS) {
            case INITIALIZE:
            case TRANSFER:
                HorizontalLinear.CS = HorizontalLinearStates.IDLE;
                if (PS != CS) {
                    time.reset();
                }
                robot.clawB.setPosition(0);
                robot.intakeY.setPosition(Globals.INTAKE_Y_TRANSFER);
                HorizontalLinear.CS = HorizontalLinearStates.EXTENDING;
                if (time.seconds() >= 1.8) {
                    robot.intakeX.setPosition(Globals.INTAKE_X_TRANSFER);
                    if (time.seconds() >= 2.2) {
                        HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                    }
                }
                break;
            case DROP:

                if (PS != ArmIntakeStates.DROP) {
                    time.reset();
                }

                robot.intakeX.setPosition(Globals.INTAKE_X_DROP);
                robot.intakeY.setPosition(Globals.INTAKE_Y_DROP);

                if (time.seconds() >= 1) {
                    IntakeClaw.CS = ClawStates.OPENED;
                    time.reset();
                    CS = ArmIntakeStates.TRANSFER;
                }
                break;
            case READ:
                HorizontalLinear.CS = HorizontalLinearStates.IDLE;
                robot.intakeY.setPosition(Globals.INTAKE_Y_READ);
                if (PS != CS) {
                    robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                }
                break;
            case CATCH:
                if (PS != ArmIntakeStates.CATCH) {
                    time.reset();
                }
                robot.intakeY.setPosition(Globals.INTAKE_Y_CATCH);

                if (time.seconds() >= 0.65) {
                    IntakeClaw.CS = ClawStates.CLOSED;
                    if (time.seconds() >= .85) {
                        telemetry.addLine("TROCA O ESTAAADO");
                        CS = ArmIntakeStates.TRANSFER;
                        IntakeSystem.CS = IntakeStates.IDLE;
                    }
                }
                break;
        }
        telemetry.addData("ArmIntakeSystem timer ", time.seconds());

        horizontalLinear.update(robot, 0);

        PS = CS;
    }

}
