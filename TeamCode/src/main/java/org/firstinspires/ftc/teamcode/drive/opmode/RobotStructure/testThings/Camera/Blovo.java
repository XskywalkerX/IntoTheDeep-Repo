package org.firstinspires.ftc.teamcode.RobotStructure.testThings.Camera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;

public class Blovo extends LinearOpMode {

    Location location = Location.OUT;
    camera camera;

    SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);
        camera = new camera();

        waitForStart();

        while (opModeIsActive()){


            switch (location) {

                case INSIDE:

                    if(gamepad1.b) {

                        camera.search();
                    }

                    break;

                case OUT:

                    robot.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));

                    break;
            }
        }
    }
}
