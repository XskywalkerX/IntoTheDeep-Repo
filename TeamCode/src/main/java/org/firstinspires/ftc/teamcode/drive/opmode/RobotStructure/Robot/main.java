package org.firstinspires.ftc.teamcode.RobotStructure.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main OpMode", group = "LogSystem")
@Disabled
public class main extends LinearOpMode {

    Robot robot;
    private static final int UPDATE_INTERVAL_MS = 10; // Update every 10 milliseconds
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new Robot(hardwareMap, gamepad1, gamepad2);

        while(!isStarted()) {
            robot.readData(telemetry);
        }

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {

            if (timer.milliseconds() > UPDATE_INTERVAL_MS) {
                robot.sendData();
                timer.reset();
            }

            telemetry.addData("FRONT LEFT DIRECTION", robot.getDrive().getFrontLeft().getDirection());
            telemetry.addData("FRONT RIGHT DIRECTION", robot.getDrive().getFrontRight().getDirection());
            telemetry.addData("BACK LEFT DIRECTION", robot.getDrive().getBackLeft().getDirection());
            telemetry.addData("BACK RIGHT DIRECTION", robot.getDrive().getBackRight().getDirection());
            telemetry.addData("rotY", robot.getIMU().getAngularOrientation().secondAngle);
            telemetry.update();

            robot.getDrive().setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (gamepad1.right_stick_y - gamepad1.right_stick_x)
                    )
            );
        }
        robot.stopServer();
    }
}