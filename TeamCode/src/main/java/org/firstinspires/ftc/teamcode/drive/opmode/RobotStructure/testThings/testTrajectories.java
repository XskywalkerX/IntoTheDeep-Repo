package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MProfile;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class testTrajectories extends LinearOpMode {

    PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
    PIDFController pidfController = new PIDFController(pidCoefficients, kV, kA, kStatic);

    MProfile mp;

    volatile public static DcMotorEx linear;
    Servo claw;
    Servo arm;

    volatile public static double kP = 0;
    volatile public static double kI = 0;
    volatile public static double kD = 0;

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 8960.0;
    public static double MAX_ACCEL = 2000.0;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        mp = new MProfile(pidCoefficients, pidfController, linear, MAX_VEL, MAX_ACCEL);

        Thread t1 = new Thread(mp);

        drive = new SampleMecanumDrive(hardwareMap);
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        drive.setPoseEstimate(new Pose2d(-36.49, 60.45, Math.toRadians(0.00)));

        Vector2d clip1 = new Vector2d(-12.52, 40.70);
        Vector2d catch1 = new Vector2d(-44.55, 60.99);
        Vector2d p1 = new Vector2d(-20, 40);
        Vector2d clip2 = new Vector2d(-12.07, 40.13);
        Vector2d catch2 = new Vector2d(-44.55, 60.21);
        Vector2d clip3 = new Vector2d(-2.51, 40.92);
        Vector2d catch3 = new Vector2d(-44.55, 60.99);
        Vector2d clip4 = new Vector2d(2.43, 40.35);

        claw.setPosition(1.0);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-36.49, 60.45, Math.toRadians(0)))
                .addSpatialMarker(clip1, () -> {
                    //clip first specimen
                    telemetry.addLine("CLIPPING FIRST SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(-7.52, 31.70))
                .addSpatialMarker(catch1, () -> {
                    //catch second specimen
                    telemetry.addLine("CATCHING SECOND SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(-44.55, 60.99))
                .addSpatialMarker(p1, () -> {
                    //move delivery to idle
                    telemetry.addLine("MOVING DELIVERY TO IDLE");
                })
                .addSpatialMarker(clip2, () -> {
                    //clip second specimen
                    telemetry.addLine("CLIPPING SECOND SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(-2.07, 32.13))
                .addSpatialMarker(catch2, () -> {
                    //catch third specimen
                    telemetry.addLine("CATCHING THIRD SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(-44.55, 60.21))
                .addSpatialMarker(p1, () -> {
                    //move delivery to idle
                    telemetry.addLine("MOVING DELIVERY TO IDLE");
                })
                .addSpatialMarker(clip3, () -> {
                    //clip third specimen
                    telemetry.addLine("CLIPPING THIRD SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(2.51, 31.92))
                .addSpatialMarker(catch3, () -> {
                    //catch fourth specimen
                    telemetry.addLine("CATCHING FOURTH SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(-44.55, 60.99))
                .addSpatialMarker(p1, () -> {
                    //move delivery to idle
                    telemetry.addLine("MOVING DELIVERY TO IDLE");
                })
                .addSpatialMarker(clip4, () -> {
                    //clip fourth specimen
                    telemetry.addLine("CLIPPING FOURTH SPECIMEN");
                })
                .lineToConstantHeading(new Vector2d(6.43, 32.35))
                .build();


        waitForStart();

        t1.start();

        if(opModeIsActive()) {
            drive.followTrajectorySequence(trajectory0);
        }
        t1.interrupt();
    }

    //delivery movements
    void c4tch() {
        claw.setPosition(1.0);
        linear.setTargetPosition(760);
    }

    void clip() {
        linear.setTargetPosition(0);
        arm.setPosition(1.0);
        claw.setPosition(0.0);
    }

    void deliveryToIdle() {
        arm.setPosition(0.0);
    }
}
