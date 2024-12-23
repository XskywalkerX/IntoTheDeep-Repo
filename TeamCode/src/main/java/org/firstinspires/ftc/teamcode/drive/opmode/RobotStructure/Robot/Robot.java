package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem.Dash;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem.SocketServer;
import org.json.JSONException;
import org.json.JSONObject;

public class Robot {

    private double[] position = {0.0, 0.0, 0.0};

    SampleMecanumDrive drive;
    Dash dash;
    SocketServer socketServer;
    BNO055IMU imu;
    VoltageSensor voltageSensor;
    LynxModule controlHub;

    public Robot(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        voltageSensor = hwMap.voltageSensor.get("Control Hub");  // Get the battery voltage sensor
        controlHub = hwMap.get(LynxModule.class, "Control Hub");

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        initServer();
    }

    private void initServer() {
        try {
            dash = new Dash();
            socketServer = new SocketServer();
            socketServer.start();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void stopServer() {
        dash.stop();
    }

    public void sendData() {
        // Retrieve IMU data
        double rotX = imu.getAngularOrientation().firstAngle;
        double rotY = imu.getAngularOrientation().secondAngle;
        double rotZ = imu.getAngularOrientation().thirdAngle;

        // Control Status data
        double batteryVoltage = voltageSensor.getVoltage();
        double temperature = controlHub.getTemperature(TempUnit.CELSIUS);

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
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        socketServer.sendData(data.toString());
    }


    // getters
    public SampleMecanumDrive getDrive() {
        return drive;
    }
}
