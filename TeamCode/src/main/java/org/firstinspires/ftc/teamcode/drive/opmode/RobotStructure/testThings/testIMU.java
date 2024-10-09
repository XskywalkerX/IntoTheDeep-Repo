package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MPU6050.MPU6050;

@TeleOp
public class testIMU extends LinearOpMode {

    MPU6050 imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(MPU6050.class, "customIMU");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        boolean boo = imu.initialize();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine(boo ? "initialized" : ":(");
            telemetry.addData("Manufacturer ID", imu.getManufacturerIDRaw());
            telemetry.addData("Pitch", imu.Pitch());
            telemetry.addData("Roll", imu.Roll());
            telemetry.addData("XACCEL", imu.getXAccel());
            telemetry.addData("BLA", imu.getXACCELB());
            telemetry.update();
        }
    }
}
