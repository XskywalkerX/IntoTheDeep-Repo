package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class debugMotors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("Left Linear Position", robot.leftLinear.getCurrentPosition());
            telemetry.addData("Right Linear Position", robot.rightLinear.getCurrentPosition());
            telemetry.update();
            robot.leftLinear.setPower(gamepad1.left_stick_y * 0.75);
            robot.rightLinear.setPower(gamepad1.left_stick_y * 0.75);
        }
    }
}
