package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.IntakeSystem;

@Config
@TeleOp
public class debugServos extends LinearOpMode {

    public static double tp = 0;

    IntakeSystem intakeSystem;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeSystem = new IntakeSystem(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            intakeSystem.moveArm(tp);
            intakeSystem.moveBox(tp);
        }
    }
}
