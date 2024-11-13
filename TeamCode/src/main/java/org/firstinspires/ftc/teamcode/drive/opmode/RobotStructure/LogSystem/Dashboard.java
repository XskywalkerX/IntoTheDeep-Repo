package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp
public class Dashboard extends LinearOpMode {

    private LogDash dash;

    @Override
    public void runOpMode() throws InterruptedException {

        if (!isStarted()) {
            try {
                dash = new LogDash();
                telemetry.addLine("it worked");
            } catch (IOException e) {
                telemetry.addData("Error", "Server couldn't start: " + e.getMessage());
            }
        }
        waitForStart();
        while(opModeIsActive()) {
            telemetry.update();
        }
        if(dash != null) {
            dash.stop();
            telemetry.addLine("Server closed");
        }
    }
}
