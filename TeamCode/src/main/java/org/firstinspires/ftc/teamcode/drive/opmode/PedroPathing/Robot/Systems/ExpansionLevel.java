package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems;

public enum ExpansionLevel {

    IDLE(0),
    BACK(5000),
    MIDDLE(5500),
    FRONT(10000);

    public final int position;

    ExpansionLevel(int position) {
        this.position = position;
    }
    public int getPosition() {
        return position;
    }
}
