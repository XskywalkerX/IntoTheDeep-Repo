package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServo extends LinearOpMode {

    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(Servo.class, "arm");

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                arm.setPosition(1.0);
            }
            if(gamepad1.b) {
                arm.setPosition(0.0);
            }
        }
    }
}
