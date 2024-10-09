package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.PIDController.PIDController;

@TeleOp
@Config
public class testMotor extends LinearOpMode {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static int tp = 0;

    DcMotorEx motor;
    PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(kP, kI, kD);
        motor = hardwareMap.get(DcMotorEx.class, "mprofile");

        controller.setCurrentPosition(motor.getCurrentPosition());
        controller.setTargetPosition(motor.getTargetPosition());

        waitForStart();
        while(opModeIsActive()) {
            motor.setTargetPosition(tp);
            motor.setPower(controller.dice());
        }
    }
}
