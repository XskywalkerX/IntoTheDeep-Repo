package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main OpMode", group = "LogSystem")
public class main extends LinearOpMode {

    Robot robot;
    private static final int UPDATE_INTERVAL_MS = 10; // Update every 10 milliseconds
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {
            if (timer.milliseconds() > UPDATE_INTERVAL_MS) {
                robot.sendData();
                timer.reset();
            }
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