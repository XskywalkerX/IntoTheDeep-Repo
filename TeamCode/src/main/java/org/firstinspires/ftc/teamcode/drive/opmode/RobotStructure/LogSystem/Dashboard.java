package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;

import java.io.IOException;

@TeleOp
public class Dashboard extends LinearOpMode {

    private LogDash logDash;
    private SocketServer socketServer;
    private VoltageSensor voltageSensor;

    private LynxModule controlHub;


    @Override
    public void runOpMode() throws InterruptedException {

        try {
            logDash = new LogDash();  // Initialize LogDash (HTTP server)
            socketServer = new SocketServer();  // Initialize WebSocket server on port 17802
            socketServer.start();  // Start the WebSocket server
        } catch (Exception e) {
            telemetry.addData("Error", "Server couldn't start: " + e.getMessage());
            return;
        }

        voltageSensor = hardwareMap.voltageSensor.get("Control Hub");  // Get the battery voltage sensor
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        waitForStart();  // Wait for the OpMode to start

        while (opModeIsActive()) {
            double batteryVoltage = voltageSensor.getVoltage();  // Get the current battery voltage
            double temp = controlHub.getTemperature(TempUnit.CELSIUS);
            String voltageString = String.valueOf(batteryVoltage);
            String tempString = String.valueOf(temp);

            // Update both the HTTP server and the WebSocket server with the battery voltage
            logDash.updateBatteryVoltage(voltageString);
            logDash.updateTemperature(tempString);
            socketServer.sendData(voltageString + "," + tempString);

            telemetry.addData("Battery Voltage", batteryVoltage);  // Show voltage on the robot's driver station
            telemetry.update();

            sleep(100);  // Update every second
        }

        // Stop the servers when the OpMode finishes
        if (logDash != null) {
            logDash.stop();
        }
        if (socketServer != null) {
            try {
                socketServer.stop();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}