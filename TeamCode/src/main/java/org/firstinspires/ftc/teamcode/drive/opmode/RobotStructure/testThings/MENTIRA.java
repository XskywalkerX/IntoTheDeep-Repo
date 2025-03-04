package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;

@TeleOp
@Disabled
public class MENTIRA extends LinearOpMode {

    SampleMecanumDrive drive;
    DcMotorEx motor;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "linear");
        servo = hardwareMap.get(Servo.class, "servo");
        drive = new SampleMecanumDrive(hardwareMap);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        while(opModeIsActive()) {

            drive.update();

            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.right_bumper) {
                servo.setPosition(1);
            } else if(gamepad1.left_bumper) {
                servo.setPosition(0);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                    -gamepad2.left_stick_y * 0.8,
                    -gamepad2.left_stick_x * 0.8,
                    (gamepad2.right_stick_y - gamepad2.right_stick_x) / 2)
            );

            telemetry.addData("LINEAR POS", motor.getCurrentPosition());
            telemetry.addData("LINEAR POWER", motor.getPower());
            telemetry.addData("LINEAR VEL", motor.getVelocity());
            telemetry.update();
        }
    }
}
