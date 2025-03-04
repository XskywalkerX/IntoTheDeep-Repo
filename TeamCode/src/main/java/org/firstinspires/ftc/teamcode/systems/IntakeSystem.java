package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ArmIntakeStates;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.HorizontalLinearStates;
import org.firstinspires.ftc.teamcode.enums.IntakeStates;

public class IntakeSystem {
    public static IntakeStates CS, PS = IntakeStates.INITIALIZE;

    HorizontalLinear horizontalLinear;
    IntakeClaw intakeClaw;
    ArmIntakeSystem armIntakeSystem;

    ElapsedTime time = new ElapsedTime();

    public IntakeSystem() {
        CS = IntakeStates.INITIALIZE;
        PS = IntakeStates.INITIALIZE;

        horizontalLinear = new HorizontalLinear();
        intakeClaw = new IntakeClaw();
        armIntakeSystem = new ArmIntakeSystem();
    }

    public void updateClawAngle(double angle) {
        armIntakeSystem.updateClawPosition(angle);
    }

    public void update(Robot robot, Telemetry telemetry, double manualPower, double clawServoPosition) {
        switch (CS) {
            case INITIALIZE:
            case IDLE:
                IntakeClaw.CS = ClawStates.CLOSED;
                HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                ArmIntakeSystem.CS = ArmIntakeStates.TRANSFER;
                break;
            case READING:
                IntakeClaw.CS = ClawStates.OPENED;
                ArmIntakeSystem.CS = ArmIntakeStates.READ;

                if (HorizontalLinear.CS == HorizontalLinearStates.RETRACTED) {
                    HorizontalLinear.CS = HorizontalLinearStates.EXTENDING;
                } else if (HorizontalLinear.CS == HorizontalLinearStates.EXTENDED) {
                    HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                }

                //telemetry.addData("Horizontal Module Intake Motor ", robot.horizontalLinear.getCurrentPosition());

                break;
            case CATCH:
                if (PS == IntakeStates.READING) {
                    if (PS != CS) {
                        ArmIntakeSystem.CS = ArmIntakeStates.CATCH;
                    }
                    if (ArmIntakeSystem.CS == ArmIntakeStates.TRANSFER) {
                        CS = IntakeStates.IDLE;
                    }
                }
                break;
            case DROP:
                HorizontalLinear.CS = HorizontalLinearStates.RETRACTING;
                ArmIntakeSystem.CS = ArmIntakeStates.DROP;
                break;
        }


        horizontalLinear.update(robot, manualPower);
        armIntakeSystem.update(robot, telemetry);
        intakeClaw.update(robot);

        telemetry.addData("INTAKE STATE", CS.name());
        telemetry.addData("HORIZONTAL STATE", HorizontalLinear.CS.name());
        telemetry.addData("ARM INTAKE STATE", ArmIntakeSystem.CS.name());
        telemetry.addData("INTAKE CLAW STATE", IntakeClaw.CS.name());

        PS = CS;
    }
}