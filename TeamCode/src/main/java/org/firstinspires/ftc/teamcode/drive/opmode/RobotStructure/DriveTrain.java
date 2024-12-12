package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Manager.Manager;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {

    public static List<Pose2d> lastPose = new ArrayList<>();
    volatile public static boolean canUp = false;
    volatile public static boolean canCatch = false;

    SampleMecanumDrive drive;
    Trajectory forward;
    Trajectory back;
    Trajectory left;
    Trajectory right;

    Trajectory line;
    TrajectorySequence spline;

    TrajectorySequence sequence;


    public DriveTrain(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
    }

    //trajectories
    public Trajectory goForward(double in, Pose2d START_POS) {

        forward = drive.trajectoryBuilder(START_POS)
                .forward(in)
                .build();

        return forward;
    }

    public Trajectory goBack(double in, Pose2d START_POS) {

        back = drive.trajectoryBuilder(START_POS)
                .back(in)
                .build();

        return back;
    }

    public Trajectory goLeft(double in, Pose2d START_POS) {

        left = drive.trajectoryBuilder(START_POS)
                .forward(in)
                .build();

        return left;
    }

    public Trajectory goRight(double in, Pose2d START_POS) {

        right = drive.trajectoryBuilder(START_POS)
                .forward(in)
                .build();

        return right;
    }

    public Trajectory goLine(double finalX, double finalY, double finalH, Pose2d START_POS) {

        line = drive.trajectoryBuilder(START_POS)
                .lineToLinearHeading(new Pose2d(finalX, finalY, Math.toRadians(finalH)))
                .build();

        return line;
    }

    public Trajectory goLine(SpatialMarker marker, double finalX, double finalY, double finalH, Pose2d START_POS) {

        line = drive.trajectoryBuilder(START_POS)
                .lineToLinearHeading(new Pose2d(finalX, finalY, Math.toRadians(finalH)))
                .addSpatialMarker(marker.getPoint(), marker.getCallback())
                .build();

        return line;
    }

    public TrajectorySequence goSpline
            (double finalX, double finalY,
             double finalH, double tangent,
             Pose2d START_POS
            ) {

        spline = drive.trajectorySequenceBuilder(START_POS)
                .splineToLinearHeading(new Pose2d(finalX, finalY, Math.toRadians(finalH)), Math.toRadians(tangent))
                .build();

        return spline;
    }

    public void turn(double degrees) {
        drive.turn(Math.toRadians(degrees));
        lastPose.add(drive.getPoseEstimate());
    }

    public void followTrajectory(Trajectory t) {
        drive.followTrajectory(t);
        lastPose.add(t.end());
    }


    public void movementSequence(List<Trajectory> movements, Pose2d START_POS) {

        if (!movements.isEmpty()) {

            TrajectorySequenceBuilder sequenceBuilder = drive.trajectorySequenceBuilder(START_POS);

            //for each trajectory inside list, add this to the sequence
            for (Trajectory movement : movements) {
                sequenceBuilder.addTrajectory(movement);
            }

            sequence = sequenceBuilder.build();
            drive.followTrajectorySequence(sequence);
            //update position
            lastPose.add(sequence.end());
        }
    }

    public void movementSequence(List<SpatialMarker> markers, List<Trajectory> movements, Pose2d START_POS) {

        if (!movements.isEmpty()) {

            TrajectorySequenceBuilder sequenceBuilder = drive.trajectorySequenceBuilder(START_POS);

            //for each trajectory inside list, add this to the sequence
            for (Trajectory movement : movements) {
                sequenceBuilder.addTrajectory(movement);
            }

            for (SpatialMarker marker : markers) {
                sequenceBuilder.addSpatialMarker(marker.getPoint(), marker.getCallback());
            }

            sequence = sequenceBuilder.build();
            drive.followTrajectorySequence(sequence);
            //update position
            lastPose.add(sequence.end());
        }
    }


    public void gamepadDrive(Gamepad gamepad1) {
        drive.setWeightedDrivePower(new Pose2d(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x));
    }


    //movements


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// TRAJECTORIES ////////////////////////////////////////////////

    public void toSubmersible() {
        Trajectory toSubmersibleZone = goLine(-23.49, -23.49, 0, lastPose());
        drive.followTrajectoryAsync(toSubmersibleZone);
        lastPose.add(toSubmersibleZone.end());
    }

    public void catchSpecimen() {
        Delivery.chamber = Delivery.Chamber.CATCH_SPECIMEN;
        Vector2d point = new Vector2d(0,0); //up linear
        SpatialMarker marker = new SpatialMarker(point, () -> {
            canUp = true;
        });
        Trajectory toObservationZone = goLine(marker,0,0, Math.toRadians(0), lastPose());
        followTrajectory(toObservationZone);
        canCatch = true;
        lastPose.add(toObservationZone.end());
    }

    public void toClip() {
        Vector2d point = new Vector2d(-20, 11);
        SpatialMarker marker = new SpatialMarker(point, () -> {
            Delivery.chamber = Delivery.Chamber.UP_LINEAR;
        });
        Trajectory toClip = goLine(marker,-23.68, -11.37, 0, lastPose());
        drive.followTrajectoryAsync(toClip);
        Delivery.chamber = Delivery.Chamber.CLIP_SPECIMEN;
        lastPose.add(toClip.end());
    }

    public void toBasket() {
        Vector2d point = new Vector2d(-45, -55);
        SpatialMarker marker = new SpatialMarker(point, () -> {
           //delivery
        });
        Trajectory toBasket = goLine(marker,-58.75, -61.4, 234.97, lastPose());
        drive.followTrajectoryAsync(toBasket);
        lastPose.add(toBasket.end());
    }

    public void toObservation() {
        Trajectory toObservation = goLine(-60.27, 59.89, 125, lastPose());
        drive.followTrajectoryAsync(toObservation);
        lastPose.add(toObservation.end());
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// SEQUENCES ////////////////////////////////////////////////////


    public void cycleBasket() {

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////getters//////////////////////////////////////////////////////
    public SampleMecanumDrive getDrive() {
        return drive;
    }
    public Pose2d lastPose() {
        return lastPose.get(lastPose.size() - 1);
    }
    public boolean isBusy() {
        return drive.isBusy();
    }
}
