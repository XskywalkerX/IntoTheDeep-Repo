package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Gustavo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/*
This is an example of a more complex path to really test the tuning. */
@Autonomous(group = "drive")
public class MYAUTO extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        Trajectory back = goBack(10, new Pose2d(11.5, -67.5, Math.toRadians(270)));

        Trajectory spline = drive.trajectoryBuilder(back.end())
                .splineToConstantHeading(new Vector2d(37, -9), Math.toRadians(270))
                .build();

        Trajectory back2 = goBack(10, spline.end());

        Trajectory forward = goForward(5, back2.end());

        Trajectory left = drive.trajectoryBuilder(forward.end())
                .strafeLeft(30)
                .build();

        Trajectory spline2 = drive.trajectoryBuilder(left.end())
                .splineToConstantHeading(new Vector2d(37, -9), Math.toRadians(270))
                .build();

        Trajectory spline3 = drive.trajectoryBuilder(spline2.end())
                .splineTo(new Vector2d(46, -9), Math.toRadians(0))
                .build();

        Trajectory right = drive.trajectoryBuilder(spline3.end())
                .strafeRight(50)
                .build();

        Trajectory spline4 = drive.trajectoryBuilder(right.end())
                .splineToLinearHeading(new Pose2d(46, -38, Math.toRadians(90)), Math.toRadians(0))
                .build();

        Trajectory back3 = goBack(25, spline4.end());

        Trajectory spline5 = drive.trajectoryBuilder(back3.end())
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Trajectory back4 = goBack(5, spline5.end());

        Trajectory forward2 = goForward(5, back4.end());

        Trajectory spline6 = drive.trajectoryBuilder(forward2.end())
                .splineToLinearHeading(new Pose2d(46, -61, Math.toRadians(90)), Math.toRadians(270))
                .build();

        Trajectory spline7 = drive.trajectoryBuilder(spline6.end())
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Trajectory back5 = goBack(5, spline7.end());

        Trajectory foward3 = goForward(5, back5.end());

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d(11.5, -67.5, Math.toRadians(270)))
                .addTrajectory(back)
                .addTrajectory(spline)
                .addTrajectory(back2)
                .addTrajectory(forward)
                .addTrajectory(left)
                .addTrajectory(spline2)
                .addTrajectory(spline3)
                .addTrajectory(right)
                .addTrajectory(spline4)
                .addTrajectory(back3)
                .addTrajectory(spline5)
                .addTrajectory(back4)
                .addTrajectory(forward2)
                .addTrajectory(spline6)
                .addTrajectory(spline7)
                .addTrajectory(back5)
                .addTrajectory(foward3)
                .turn(Math.toRadians(90))
                .build();

        drive.followTrajectorySequence(sequence);
    }

    public Trajectory goBack(double distance, Pose2d startPose) {

        return drive.trajectoryBuilder(startPose).back(distance).build();
    }
    public Trajectory goForward(double distance, Pose2d startPose) {
        return drive.trajectoryBuilder(startPose).forward(distance).build();
    }
}