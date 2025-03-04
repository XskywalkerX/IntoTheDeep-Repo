package org.firstinspires.ftc.teamcode.RobotStructure.Robot;

public enum State {
    IDLE("IDLE"),
    LIFTED("LIFTED");

    public final String stateName;

    public static State getState(String stateName) {
        State match = null;

        for (State state : State.values()) {
            if (state.stateName.equals(stateName)) {
                match = state;
            }
        }
        return match;
    }

    private State(String stateName) {
        this.stateName = stateName;
    }
}
