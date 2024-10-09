package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.clipped;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.dunked;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.DriveTrain.lastPose;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.CLOSED_ITK;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.UP;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.expanded;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.hasSample;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.yellowOn;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "IF YOU WANNA ROCK 'N ROLL")
public class Auto extends LinearOpMode {

    enum Mode {
        SEARCHING,
        FOUND,
        GO_TO_SUB,
        GO_TO_BASKET,
        GO_TO_OBSERVATION_ZONE,
        GO_TO_CLIP,
        DELIVERY_SAMPLE,
        IDLE
    }

    Webcam webcam;

    DriveTrain drive;
    Pipeline mask;
    Intake intake;
    Delivery delivery;
    Mode mode = Mode.IDLE;

    Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);
        delivery = new Delivery(hardwareMap);
        webcam = new Webcam(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        mask = new Pipeline();

        webcam.setPipeline(mask);

        Pose2d START = new Pose2d(-60.06, -12.69, 0);
        lastPose.add(START);

        webcam.startStreaming();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam.getWebcam(), 30);

        waitForStart();
        if(isStopRequested()) return;

        mode = Mode.GO_TO_CLIP;
        drive.toClip();
        UP = true;

        while (opModeIsActive() && !isStopRequested()) {

            expanded = intake.getExpansion().getCurrentPosition() > CLOSED_ITK + 30;

            switch(mode) {

                ///////////////////CLIPPING///////////////////////////////////////
                case GO_TO_CLIP:

                    if(!drive.isBusy() && clipped) {
                        mode = Mode.GO_TO_SUB;
                        drive.toSubmersible();
                    }
                    break;

                /////////////////TO SUBMERSIBLE ZONE//////////////////////////////
                case GO_TO_SUB:
                    if(!drive.isBusy()) {
                        mode = Mode.SEARCHING;
                    }
                    break;

                ///////////////HUNTING SAMPLE/////////////////////////////////////
                case SEARCHING:
                    findYellow();
                    break;

                //////////////CATCH SAMPLE///////////////////////////////////////

                case FOUND:

                    //moveIntake
                    lastPose.add(drive.getDrive().getPoseEstimate());
                    intake.catchSample();
                    if(hasSample) {
                        intake.returnIntake();
                        if(!expanded) {
                            mode = Mode.GO_TO_BASKET;
                            drive.toBasket();
                        }
                    }
                    break;

                ////////////DUNK///////////////////////////////////////////////
                case GO_TO_BASKET:

                    if(dunked) {
                        mode = Mode.GO_TO_SUB;
                        drive.toSubmersible();
                    }
                    break;

                ////////////IDLE/////////////////////////////////////////////////
                case IDLE:

                    break;
            }
            drive.getDrive().update();
            intake.update();
            delivery.update();

            intake.telemetry(telemetry);
            delivery.telemetry(telemetry);
        }
    }


    private void findYellow() {
        if (yellowOn) {
            drive.getDrive().setDrivePower(new Pose2d(0, 0, Math.toRadians(0)));
            mode = Mode.FOUND;

        } else {
            drive.getDrive().setWeightedDrivePower(new Pose2d(
                    0,
                    drive.getDrive().getPoseEstimate().getY() - 10,
                    0
            ));
        }
    }
}
