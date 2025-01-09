package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Robot;

import android.app.ActivityManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem.Dash;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem.SocketServer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.java_websocket.WebSocket;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.net.InetAddress;
import java.util.Iterator;

public class Robot {

    private double[] position = {0.0, 0.0, 0.0};

    HardwareMap hwMap;

    State state;

    SampleMecanumDrive drive;
    Dash dash;
    SocketServer socketServer;
    SocketServer socketServer2;
    BNO055IMU imu;
    VoltageSensor voltageSensor;
    LynxModule controlHub;

    Gamepad gamepad1, gamepad2;

    public Robot(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {

        this.hwMap = hwMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        state = State.IDLE;

        drive = new SampleMecanumDrive(hwMap);
        voltageSensor = hwMap.voltageSensor.get("Control Hub");  // Get the battery voltage sensor
        controlHub = hwMap.get(LynxModule.class, "Control Hub");

        imu = drive.getImu();

        initServer();
    }

    private void initServer() {
        try {
            dash = new Dash();
            socketServer = new SocketServer(17802);
            socketServer2 = new SocketServer(17803);
            socketServer.start();
            socketServer2.start();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void stopServer() {
        dash.stop();
    }

    public void readData(Telemetry telemetry) {
        String jsonData = readJSONFromFile();
        if (jsonData != null) {
            try {
                // Parse the JSON string
                JSONObject config = new JSONObject(jsonData);

                setMotorDirection(
                        drive.getFrontLeft(),
                        config.optString("frontLeftDir", "FORWARD"),
                        telemetry);
                setMotorDirection(drive.getFrontRight(),
                        config.optString("frontRightDir", "FORWARD"),
                        telemetry);
                setMotorDirection(drive.getBackLeft(),
                        config.optString("backLeftDir", "FORWARD"),
                        telemetry);
                setMotorDirection(drive.getBackRight(),
                        config.optString("backRightDir", "FORWARD"),
                        telemetry);

                setEncoderDirection(drive.getLeftEncoder(),
                        config.optString("leftdeadWheelDir", "FORWARD"));
                setEncoderDirection(drive.getRightEncoder(),
                        config.optString("rightdeadWheelDir", "FORWARD"));
                setEncoderDirection(drive.getFrontEncoder(),
                        config.optString("frontdeadWheelDir", "FORWARD"));

                // Example: Print keys from the JSON to telemetry
                telemetry.addLine("Received Configuration:");
                Iterator<String> keys = config.keys();
                while (keys.hasNext()) {
                    String key = keys.next();
                    telemetry.addData(key, config.get(key));
                }
                telemetry.update();
            } catch (Exception e) {
                telemetry.addLine("Error parsing JSON.");
                telemetry.update();
                e.printStackTrace();
            }
        } else {
            telemetry.addLine("No JSON file found.");
            telemetry.update();
        }
    }

    private String readJSONFromFile() {
        StringBuilder jsonData = new StringBuilder();
        try {
            File file = new File("/sdcard/TAMOPORCA/config.json");  // File path on Control Hub
            if (file.exists()) {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                String line;
                while ((line = reader.readLine()) != null) {
                    jsonData.append(line);  // Append each line to the StringBuilder
                }
                reader.close();
                return jsonData.toString();
            } else {
                System.out.println("File not found: " + file.getAbsolutePath());
                return null;
            }
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error reading JSON from file");
            return null;
        }
    }


    public void sendData() {

        StringBuilder warnings = new StringBuilder();

        if (!checkEncoder(drive.getFrontEncoder())) {
            warnings.append("Front Encoder disconnected.\n");
        }
        if (!checkEncoder(drive.getLeftEncoder())) {
            warnings.append("Left Encoder disconnected.\n");
        }
        if (!checkEncoder(drive.getRightEncoder())) {
            warnings.append("Right Encoder disconnected.\n");
        }
        if(gamepad1.getGamepadId() == -1) {
            warnings.append("Gamepad 1 disconnected.\n");
        }
        if(gamepad2.getGamepadId() == -1) {
            warnings.append("Gamepad 2 disconnected.\n");
        }

        // Retrieve IMU data
        double rotX = imu.getAngularOrientation().firstAngle;
        double rotY = imu.getAngularOrientation().secondAngle;
        double rotZ = imu.getAngularOrientation().thirdAngle;

        //change state
        state = (rotY > 40 || rotY < -40) ? State.LIFTED : State.IDLE;

        // Control Status data
        double batteryVoltage = voltageSensor.getVoltage();
        double temperature = controlHub.getTemperature(TempUnit.CELSIUS);

        if (batteryVoltage < 12.0) {
            warnings.append("Warning: Battery voltage is low (" + batteryVoltage + "V).\n");
        }

        if (warnings.length() == 0) {
            warnings.append("All systems operational.\n");
        }

        // Chassis motors data
        double flPower = drive.getFrontLeft().getPower() * 100;
        double frPower = drive.getFrontRight().getPower() * 100;
        double blPower = drive.getBackLeft().getPower() * 100;
        double brPower = drive.getBackRight().getPower() * 100;

        double[][] rotationMatrix = {
                {0, 1, 0},  // i-axis
                {0, 0, 1},  // j-axis
                {-1, 0, 0}  // k-axis
        };

        double memoryUsage = getMemoryUsage();
        double cpuUsage = getCpuUsage();


        double ping = getPing("192.168.43.1");

        JSONObject data = new JSONObject();
        try {
            data.put("posX", position[0]);
            data.put("posY", position[1]);
            data.put("posZ", position[2]);
            data.put("rotX", rotX);
            data.put("rotY", rotY);
            data.put("rotZ", rotZ);

            data.put("i_x", rotationMatrix[0][0]);
            data.put("i_y", rotationMatrix[1][0]);
            data.put("i_z", rotationMatrix[2][0]);

            data.put("j_x", rotationMatrix[0][1]);
            data.put("j_y", rotationMatrix[1][1]);
            data.put("j_z", rotationMatrix[2][1]);

            data.put("k_x", rotationMatrix[0][2]);
            data.put("k_y", rotationMatrix[1][2]);
            data.put("k_z", rotationMatrix[2][2]);

            data.put("battery", batteryVoltage);
            data.put("temperature", temperature);

            data.put("fl_power", Math.abs(flPower));
            data.put("fr_power", Math.abs(frPower));
            data.put("bl_power", Math.abs(blPower));
            data.put("br_power", Math.abs(brPower));

            data.put("cpu", cpuUsage);
            data.put("memory", memoryUsage);
            data.put("ping", ping);

            data.put("state", state.stateName);

            data.put("Warning", warnings);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        socketServer.sendData(data.toString());
    }


    // getters
    public SampleMecanumDrive getDrive() {
        return drive;
    }


    // setters
    private void setMotorDirection(DcMotor motor, String direction, Telemetry telemetry) {
        if (direction.equalsIgnoreCase("FORWARD")) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction.equalsIgnoreCase("REVERSE")) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            telemetry.addLine("Invalid motor direction: " + direction + ". Using default FORWARD.");
            telemetry.update();
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    private void setEncoderDirection(Encoder encoder, String direction) {
        if (direction.equalsIgnoreCase("FORWARD")) {
            System.out.println("blablabla");
            encoder.setDirection(Encoder.Direction.FORWARD);
        } else if (direction.equalsIgnoreCase("REVERSE")) {
            encoder.setDirection(Encoder.Direction.REVERSE);
        } else {
            encoder.setDirection(Encoder.Direction.FORWARD);
        }
    }

    public double getMemoryUsage() {
        ActivityManager activityManager = (ActivityManager) hwMap.appContext.getSystemService(android.content.Context.ACTIVITY_SERVICE);
        ActivityManager.MemoryInfo memoryInfo = new ActivityManager.MemoryInfo();
        activityManager.getMemoryInfo(memoryInfo);

        long totalMemory = memoryInfo.totalMem / (1024 * 1024); // Convert to MB
        long availableMemory = memoryInfo.availMem / (1024 * 1024); // Convert to MB
        long usedMemory = totalMemory - availableMemory;

        return usedMemory;
    }

    // Get CPU usage
    public double getCpuUsage() {
        try {
            RandomAccessFile reader = new RandomAccessFile("/proc/stat", "r");
            String[] firstLine = reader.readLine().split("\\s+");
            long idleTime1 = Long.parseLong(firstLine[4]);
            long totalTime1 = 0;

            for (int i = 1; i < firstLine.length; i++) {
                totalTime1 += Long.parseLong(firstLine[i]);
            }

            // Wait a short time to sample CPU usage
            Thread.sleep(100);

            reader.seek(0);
            String[] secondLine = reader.readLine().split("\\s+");
            long idleTime2 = Long.parseLong(secondLine[4]);
            long totalTime2 = 0;

            for (int i = 1; i < secondLine.length; i++) {
                totalTime2 += Long.parseLong(secondLine[i]);
            }

            reader.close();

            long totalDelta = totalTime2 - totalTime1;
            long idleDelta = idleTime2 - idleTime1;

            double cpuUsage = (1.0 - (double) idleDelta / totalDelta) * 100.0;
            return cpuUsage;
        } catch (Exception e) {
            e.printStackTrace();
            return 0.0;
        }
    }

    public static long getPing(String ipAddress) {
        try {
            InetAddress inet = InetAddress.getByName(ipAddress);
            long startTime = System.nanoTime();
            boolean reachable = inet.isReachable(1000); // Timeout of 1000ms
            long endTime = System.nanoTime();

            if (reachable) {
                return (endTime - startTime) / 1_000_000; // Convert nanoseconds to milliseconds
            } else {
                return -1; // Return -1 if not reachable
            }
        } catch (Exception e) {
            return -1; // Return -1 for errors
        }
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    //checkers
    private boolean checkEncoder(Encoder encoder) {
        if(drive.getFrontLeft().getPower() != 0 || drive.getBackLeft().getPower() != 0
        || drive.getFrontRight().getPower() != 0 || drive.getBackRight().getPower() != 0) {
            return encoder.getCorrectedVelocity() != 0;
        } else {
            return true;
        }
    }

    private boolean checkMotor(DcMotorEx motor) {
        if(motor.getPower() != 0) {
            return motor.getVelocity() != 0;
        } else {
            return true;
        }
    }
}