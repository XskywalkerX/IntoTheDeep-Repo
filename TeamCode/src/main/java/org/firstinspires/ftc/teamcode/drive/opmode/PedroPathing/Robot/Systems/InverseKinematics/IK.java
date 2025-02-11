package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.InverseKinematics;

public class IK {

    double x, y, z, l1, l2 = 0;

    public IK(double x, double y, double z, double l1, double l2) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.l1 = l1;
        this.l2 = l2;
    }

    public void setCoordinates(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    double r() {
        return Math.sqrt((x * x) + (y * y));
    }

    double d() {
        return Math.sqrt((r() * r()) + (z * z));
    }

    double alpha() {
        return Math.acos(((l1 * l1) + (d() * d()) - (l2 * l2)) / (2 * l1 * d()));
    }

    double beta() {
        return Math.acos(((l1 * l1) + (l2 * l2) - (d() * d())) / (2 * l1 * l2));
    }

    double gama() {
        return Math.atan(z / r());
    }

    public double theta1() {
        return Math.atan(y / x);
    }

    public double theta2() {
        return gama() - alpha();
    }
}
