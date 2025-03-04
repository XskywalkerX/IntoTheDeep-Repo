package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
@Disabled
public class DeadWheelsDebugger extends LinearOpMode {

    Encoder front, left, right;

    @Override
    public void runOpMode() throws InterruptedException {

        front = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        left = new Encoder(hardwareMap.get(DcMotorEx.class, "motor_linear_front"));
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        left.setDirection(Encoder.Direction.FORWARD);
        right.setDirection(Encoder.Direction.FORWARD);
        front.setDirection(Encoder.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("f", front.getCurrentPosition());
            telemetry.addData("r", right.getCurrentPosition());
            telemetry.addData("l", left.getCurrentPosition());
            telemetry.update();
        }
    }
}
