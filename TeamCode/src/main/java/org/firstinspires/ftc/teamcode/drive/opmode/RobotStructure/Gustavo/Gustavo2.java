package org.firstinspires.ftc.teamcode.RobotStructure.Gustavo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;

@Autonomous
@Disabled
public class Gustavo2 extends LinearOpMode {
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory Submersible = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        robot.followTrajectory(Submersible);
    }
}
