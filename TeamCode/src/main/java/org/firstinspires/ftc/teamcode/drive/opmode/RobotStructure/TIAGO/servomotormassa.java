package org.firstinspires.ftc.teamcode.RobotStructure.TIAGO;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Disabled
public class servomotormassa extends LinearOpMode {
    DcMotorEx servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(DcMotorEx.class, "backLeft");

        waitForStart();

        while (opModeIsActive()){
            servo.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
