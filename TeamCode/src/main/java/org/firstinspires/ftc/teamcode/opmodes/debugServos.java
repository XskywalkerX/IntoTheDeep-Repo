package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;


@Config

@TeleOp
public class debugServos extends LinearOpMode {

    public static double intakeX, intakeY, intakeClaw = 0.35;
    public static double deliveryX, deliveryY = 0;
    public static double clawB, clawIntake, clawDelivery = 0;


    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot robot = new Robot(hardwareMap);

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            robot.deliveryX.setPosition(deliveryX);
        }
    }
}
