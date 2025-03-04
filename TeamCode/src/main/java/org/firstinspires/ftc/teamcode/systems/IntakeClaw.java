package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.HorizontalLinearStates;

@Config
public class IntakeClaw {
    public static ClawStates CS = ClawStates.INITIALIZE, PS = ClawStates.INITIALIZE;

    public static double closed = .75;
    public static double opened = 0;

    public IntakeClaw() {
        CS = ClawStates.INITIALIZE;
        PS = ClawStates.INITIALIZE;
    }

    public void update(Robot robot) {
        if (PS != CS || CS == ClawStates.INITIALIZE || CS == ClawStates.CLOSED || CS == ClawStates.OPENED) {
            switch (CS) {
                case INITIALIZE:
                case CLOSED: {
                    robot.intakeClaw.setPosition(closed);
                    break;
                }
                case OPENED:
                {
                    robot.intakeClaw.setPosition(opened);
                    break;
                }
            }
        }
        PS = CS;
    }

    public void setState(ClawStates state) {
        CS = state;
    }
}