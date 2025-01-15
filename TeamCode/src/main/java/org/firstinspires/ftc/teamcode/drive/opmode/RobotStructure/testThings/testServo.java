package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testServo extends LinearOpMode {

    Servo arm;

    public static double tp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(Servo.class, "arm");

        waitForStart();
        while(opModeIsActive()) {
            arm.setPosition(tp);
        }
    }
}
