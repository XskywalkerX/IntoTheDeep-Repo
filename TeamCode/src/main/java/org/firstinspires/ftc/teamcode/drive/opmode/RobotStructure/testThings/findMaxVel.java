package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class findMaxVel extends LinearOpMode {

    DcMotorEx mprofile;
    double velocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        mprofile = hardwareMap.get(DcMotorEx.class, "mprofile");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while(opModeIsActive()) {

            ElapsedTime now = new ElapsedTime();

            velocity = mprofile.getVelocity();

            double dt = timer.seconds() - now.seconds();
            double accel = velocity * dt;

            mprofile.setPower(1);

            telemetry.addData("VEL", velocity);
            telemetry.addData("ACCEL", accel);
            telemetry.addData("dT", dt);
            telemetry.addData("1 SEC", accel / dt);
            telemetry.update();

            timer.reset();
        }
    }
}
