package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Robot;

public enum linearLevel {

    GROUND(0),
    LOW_BASKET(5000),
    CLIP(5500),
    HIGH_BASKET(10000);

    public final int position;

    linearLevel(int position) {
        this.position = position;
    }
    public int getPosition() {
        return position;
    }
}