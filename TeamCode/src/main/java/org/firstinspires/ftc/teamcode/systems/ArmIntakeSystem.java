package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
public class ArmIntakeSystem {

    public static ArmIntakeStates CS = ArmIntakeStates.INITIALIZE,
            PS = ArmIntakeStates.INITIALIZE;

    ElapsedTime time = new ElapsedTime();

    double clawBAngle = 0;

    public ArmIntakeSystem() {
        CS = ArmIntakeStates.INITIALIZE;
        PS = ArmIntakeStates.INITIALIZE;
    }

    public void updateClawAngle(double angle) {
        clawBAngle = (angle) / 180;
    }

    public void updateClawPosition(double angle) {
        clawBAngle = angle;
    }

    public void update(Robot robot, Telemetry telemetry) {
        robot.clawB.setPosition(clawBAngle);
        switch (CS) {
            case INITIALIZE:
            case TRANSFER:
                robot.intakeX.setPosition(Globals.INTAKE_X_TRANSFER);
                robot.intakeY.setPosition(Globals.INTAKE_Y_TRANSFER);
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
                robot.intakeY.setPosition(Globals.INTAKE_Y_READ);
                robot.intakeX.setPosition(Globals.INTAKE_X_READ);
                break;
            case CATCH:

                if (PS != ArmIntakeStates.CATCH) {
                    time.reset();
                }

                robot.clawB.setPosition(clawBAngle);

                robot.intakeY.setPosition(Globals.INTAKE_Y_CATCH);
                robot.intakeX.setPosition(Globals.INTAKE_X_CATCH);

                if (time.seconds() >= 1) {
                    IntakeClaw.CS = ClawStates.CLOSED;
                    if (time.seconds() >= 1.5) {
                        CS = ArmIntakeStates.TRANSFER;
                        time.reset();
                    }
                }
                break;
        }
        PS = CS;
    }

}
