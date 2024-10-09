package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Gustavo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.PoseStorage.PoseStorage;

@Config
@Autonomous(name = "Gustavoautonomous")
public class Gustavo extends LinearOpMode {
    SampleMecanumDrive robot;
    //Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        //servo = hardwareMap.get(Servo.class, "servo");
        Pose2d START = new Pose2d(0, 0, 0);
        robot.setPoseEstimate(START);


        waitForStart();

        while (opModeIsActive()) {
            Trajectory toSubmersible = robot.trajectoryBuilder(robot.getPoseEstimate())
                    //.lineToSplineHeading(new Pose2d(-33, 39, Math.toRadians(138)))
                    .lineToSplineHeading(new Pose2d(56, 0, Math.toRadians(0)))
                    .build();


            robot.followTrajectory(toSubmersible);

            PoseStorage.currentPose = toSubmersible.end();

            Trajectory toBasket = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(11, 54, Math.toRadians(124)))
                    .build();


            robot.followTrajectory(toBasket);

        }
    }
}
