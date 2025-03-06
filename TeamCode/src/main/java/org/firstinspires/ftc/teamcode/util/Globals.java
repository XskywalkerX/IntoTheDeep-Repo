package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

@Config
public class Globals {

    public static double INTAKE_X_SAFE_MIN = 0.25;
    public static double INTAKE_X_SAFE_MAX = 0.51;

    public static double CAMERA_Y_CATCH_SETPOINT = 235;
    public static double CAMERA_X_CATCH_SETPOINT = 195;

    public static double INTAKE_CLAW_IDLE = 0.4;

    public static double INTAKE_Y_CATCH = 1;
    public static double INTAKE_X_CATCH = 0.7;

    public static double INTAKE_X_TRANSFER = 0.63;
    public static double INTAKE_Y_TRANSFER = 0;

    public static double INTAKE_X_DROP = 0.4;
    public static double INTAKE_Y_DROP = 0.5;

    public static double INTAKE_Y_READ = 0.3;
    public static double INTAKE_X_READ = 0.35;

    public static int MAX_EXPANDED_INTAKE = 120;
    public static int MAX_RETRACTED_INTAKE = 0;
    public static double MAX_INTAKE_POWER = 0.3;


    public static double SERVO_MIN = -350;
    public static double SERVO_MAX = 250;

    public static Pose OBSERVATION_ZONE = new Pose(-44.55, 60.45, Math.toRadians(180));

    public static Pose FIRST_SPECIMEN = new Pose(-5.52, 40.13, Math.toRadians(180));
    public static Pose SECOND_SPECIMEN = new Pose(-12.07, 40.13, Math.toRadians(180));
    public static Pose THIRD_SPECIMEN = new Pose(-2.51, 40.92, Math.toRadians(180));
    public static Pose FOURTH_SPECIMEN = new Pose(2.43, 40.35, Math.toRadians(180));

    public static Pose FLOOR_SAMPLE = new Pose(-44.55, 60.21, Math.toRadians(180));

    public static Path FIRST_SPEC, SEC_SPEC, THIRD_SPEC, FOURTH_SPEC, SAMPLE, CATCH;

    public static void BUILD_PATHS(Follower follower) {
        FIRST_SPEC = new Path(new BezierLine(new Point(follower.getPose()), new Point(FIRST_SPECIMEN)));
        SEC_SPEC = new Path(new BezierLine(new Point(follower.getPose()), new Point(SECOND_SPECIMEN)));
        THIRD_SPEC = new Path(new BezierLine(new Point(follower.getPose()), new Point(THIRD_SPECIMEN)));
        FOURTH_SPEC = new Path(new BezierLine(new Point(follower.getPose()), new Point(FOURTH_SPECIMEN)));
        SAMPLE = new Path(new BezierLine(new Point(follower.getPose()), new Point(FLOOR_SAMPLE)));
        CATCH = new Path(new BezierLine(new Point(follower.getPose()), new Point(OBSERVATION_ZONE)));
    }
}
