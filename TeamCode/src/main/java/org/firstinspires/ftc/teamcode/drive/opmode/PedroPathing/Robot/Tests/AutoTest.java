package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.LinearLevel;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.LConstants;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {

    public static double VALUE = 3;

    Rev2mDistanceSensor distanceSensor;

    LinearLevel level = LinearLevel.GROUND;

    Servo claw;
    Servo arm;

    RevTouchSensor sensor;

    int pathState = 0;

    Follower follower;
    Timer pathTimer = new Timer();

    Path chamber1, chamber2, chamber3, chamber4, obsZone1, obsZone2, obsZone3;

    Pose startPose = new Pose(-36.49, 60.45, Math.toRadians(180));

    Pose clip1 = new Pose(-5.52, 40.13, Math.toRadians(180));
    Pose catch1 = new Pose(-44.55, 60.45, Math.toRadians(180));
    Pose p1 = new Pose(-20, 40, Math.toRadians(180));
    Pose clip2 = new Pose(-12.07, 40.13, Math.toRadians(180));
    Pose catch2 = new Pose(-44.55, 60.21, Math.toRadians(180));
    Pose clip3 = new Pose(-2.51, 40.92, Math.toRadians(180));
    Pose catch3 = new Pose(-44.55, 60.99, Math.toRadians(180));
    Pose clip4 = new Pose(2.43, 40.35, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);

        pathState = 0;
        follower = new Follower(hardwareMap);
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        sensor = hardwareMap.get(RevTouchSensor.class, "wallLeft");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "wallRight");

        FConstants.readData();
        LConstants.readData();

        follower.setStartingPose(startPose);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        buildPaths();
        //arm.setPosition(0.4);
        claw.setPosition(1.0);

        while (!isStarted()) {
            telemetry.addData("SENSOR", sensor.getValue());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("STATE", pathState);
            telemetry.addData("SENSOR", sensor.getValue());
            telemetry.addData("DISTANCE", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            follower.update();
            autonomousPathUpdate();
        }
    }

    public void buildPaths() {
        chamber1 = new Path(new BezierLine(
                new Point(follower.getPose()),
                new Point(clip1)));
        chamber1.setLinearHeadingInterpolation(follower.getPose().getHeading(), clip1.getHeading());

        obsZone1 = new Path(new BezierLine(
                new Point(follower.getPose()),
                new Point(catch1)
        ));
        obsZone1.setLinearHeadingInterpolation(follower.getPose().getHeading(), catch1.getHeading());

        chamber2 = new Path(new BezierLine(
                new Point(catch1),
                new Point(clip2)
        ));
        chamber2.setLinearHeadingInterpolation(catch1.getHeading(), clip2.getHeading());

        obsZone2 = new Path(new BezierLine(
                new Point(follower.getPose()),
                new Point(catch2)
        ));
        obsZone2.setLinearHeadingInterpolation(follower.getPose().getHeading(), catch2.getHeading());

        chamber3 = new Path(new BezierLine(
                new Point(follower.getPose()),
                new Point(clip3)
        ));
        chamber3.setLinearHeadingInterpolation(follower.getPose().getHeading(), clip3.getHeading());

        obsZone3 = new Path(new BezierLine(
                new Point(clip3),
                new Point(catch3)
        ));
        obsZone3.setLinearHeadingInterpolation(follower.getPose().getHeading(), catch3.getHeading());

        chamber4 = new Path(new BezierLine(
                new Point(follower.getPose()),
                new Point(clip4)
        ));
        chamber4.setLinearHeadingInterpolation(follower.getPose().getHeading(), clip4.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                buildPaths();
                follower.followPath(chamber1);
                setPathState(1);
                break;

            case 1:
                //arm.setPosition(0.6);
                if (!follower.isBusy()) {
                    buildPaths();
                    //arm.setPosition(0.3);
                    level = LinearLevel.CLIP;
                    follower.followPath(obsZone1);
                    setPathState(2);
                }
                break;

            case 2:
                level = LinearLevel.GROUND;
                if (sensor.isPressed()) {
                    buildPaths();
                    follower.followPath(chamber2);
                    setPathState(3);
                }
                break;

            case 3:

                if (!follower.isBusy()) {

                    if (distanceSensor.getDistance(DistanceUnit.INCH) > 6.5 ||
                            distanceSensor.getDistance(DistanceUnit.INCH) < 6) {
                        double distance = distanceSensor.getDistance(DistanceUnit.INCH) - VALUE;

                        Pose alignPose = new Pose(
                                follower.getPose().getX(),
                                follower.getPose().getY() - distance,
                                Math.toRadians(180)
                        );

                        Path align = new Path(
                                new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(alignPose)
                                )
                        );
                        align.setLinearHeadingInterpolation(follower.getPose().getHeading(), alignPose.getHeading());

                        follower.followPath(align);
                    } else {
                        buildPaths();
                        follower.followPath(obsZone2);
                        setPathState(4);
                    }
                }
                break;

            case 4:

                if (!follower.isBusy()) {
                    buildPaths();
                    follower.followPath(chamber3);
                    setPathState(5);
                }

                break;

            case 5:

                if (!follower.isBusy()) {

                    if (distanceSensor.getDistance(DistanceUnit.INCH) > 6.5 ||
                            distanceSensor.getDistance(DistanceUnit.INCH) < 6) {
                        double distance = distanceSensor.getDistance(DistanceUnit.INCH) - VALUE;

                        Pose alignPose = new Pose(
                                follower.getPose().getX(),
                                follower.getPose().getY() - distance,
                                Math.toRadians(180)
                        );

                        Path align = new Path(
                                new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(alignPose)
                                )
                        );
                        align.setLinearHeadingInterpolation(follower.getPose().getHeading(), alignPose.getHeading());

                        follower.followPath(align);
                    } else {
                        buildPaths();
                        follower.followPath(obsZone3);
                        setPathState(6);
                    }
                }

                break;

            case 6:

                if (!follower.isBusy()) {
                    buildPaths();
                    follower.followPath(chamber4);
                    setPathState(7);
                }

                break;

            case 7:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
