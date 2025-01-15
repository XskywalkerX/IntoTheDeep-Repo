package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.LConstants;

@Autonomous(group = "test")
public class TonhoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);

        FConstants.readData();
        LConstants.readData();

        waitForStart();

        Pose startPose = new Pose(0, 0, 0);
        Pose endPose = new Pose(20, 0, 0);

        Path target = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(endPose)
                )
        );
        target.setTangentHeadingInterpolation();


        if (opModeIsActive()) {
            follower.update();
            follower.followPath(target);
        }
    }
}
