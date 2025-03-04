package org.firstinspires.ftc.teamcode.drive.Systems;

public enum LinearLevel {

    GROUND(0),
    LOW_BASKET(2000),
    CLIP(7000),
    HIGH_BASKET(4500);

    public final int position;

    LinearLevel(int position) {
        this.position = position;
    }
    public int getPosition() {
        return position;
    }
}
