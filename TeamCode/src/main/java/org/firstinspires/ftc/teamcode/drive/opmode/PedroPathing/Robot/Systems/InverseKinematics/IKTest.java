package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.InverseKinematics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "IK Test", group = "INVERSE KINEMATICS")
public class IKTest extends LinearOpMode {

    public static double x, y, z = 0;

    double l1 = 10;
    double l2 = 20;

    Servo armX, armY;

    @Override
    public void runOpMode() throws InterruptedException {
        IK ik = new IK(x, y, z, l1, l2);

        armX = hardwareMap.get(Servo.class, "armX");
        armY = hardwareMap.get(Servo.class, "armY");

        waitForStart();

        while(opModeIsActive()) {

            ik.setCoordinates(x, y, z);

            armX.setPosition(ik.theta1() / Math.toRadians(360));
            armY.setPosition(ik.theta2() / Math.toRadians(360));

            telemetry.addData("THETA 1", ik.theta1());
            telemetry.addData("THETA 2", ik.theta2());
            telemetry.addData("POS X", armX.getPosition());
            telemetry.addData("POS Y", armY.getPosition());
            telemetry.update();
        }
    }
}
