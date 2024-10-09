package org.firstinspires.ftc.teamcode.models;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Rectangle {

    private double x1 = 0;
    private double x2 = 0;
    private double y1 = 0;
    private double y2 = 0;
    private double width = 0;
    private double height = 0;

    public Rectangle(double x, double y, double width, double height, boolean ignore) {
        this.x1 = x;
        this.y1 = y;
        this.x2 = x + width;
        this.y2 = y + width;
        this.width = width;
        this.height = height;
    }

    public Rectangle(double x1, double x2, double y1, double y2) {
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
        this.width = x2 - x1;
        this.height = y2 - y1;
    }

    public boolean contains(double x, double y) {
        return (y > y1 && y < y2) && (x > x1 && x < x2);
    }

    public void drawRectangle(FtcDashboard ftcDashboard) {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay()
                .setStroke("blue")
                .strokeRect(this.x1, this.y1, this.width, this.height);
        ftcDashboard.sendTelemetryPacket(telemetryPacket);
    }

    public double getX1() {
        return x1;
    }

    public double getX2() {
        return x2;
    }

    public double getY1() {
        return y1;
    }

    public double getY2() {
        return y2;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }
}
