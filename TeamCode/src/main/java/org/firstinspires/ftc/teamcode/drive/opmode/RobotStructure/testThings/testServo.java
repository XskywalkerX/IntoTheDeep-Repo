package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
@Disabled
public class testServo extends LinearOpMode {

    Servo arm;
    DcMotorEx motor;

    public static double tp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(Servo.class, "arm");
        motor = hardwareMap.get(DcMotorEx.class, "linear");

        arm.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(gamepad1.right_trigger);
            arm.setPosition(tp);
        }
    }
}
