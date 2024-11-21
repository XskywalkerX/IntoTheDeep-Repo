package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
import org.json.JSONException;
import org.json.JSONObject;

@TeleOp(name = "IMU WebSocket OpMode", group = "Sensor")
public class IMUOpMode extends LinearOpMode {

    SampleMecanumDrive drive;

    private BNO055IMU imu;
    Dash dash;
    SocketServer client;
    private double[] position = {0.0, 0.0, 0.0}; // Placeholder for any position tracking, if applicable
    private ElapsedTime timer = new ElapsedTime();
    private static final int UPDATE_INTERVAL_MS = 10; // Update every 10 milliseconds

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating IMU...");
            telemetry.update();
            sleep(50);
        }

        // Connect WebSocket
        // connectWebSocket();
        try {
            dash = new Dash();
            client = new SocketServer();
            client.start();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            if (timer.milliseconds() > UPDATE_INTERVAL_MS) {
                sendIMUData();
                timer.reset();
            }

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));
        }

        // Close WebSocket connection if open
        dash.stop();
    }

    private void sendIMUData() {
        // Retrieve IMU data
        double rotX = imu.getAngularOrientation().firstAngle;
        double rotY = imu.getAngularOrientation().secondAngle;
        double rotZ = imu.getAngularOrientation().thirdAngle;

        double[][] rotationMatrix = {
                {0, 1, 0},  // i-axis
                {0, 0, 1},  // j-axis
                {-1, 0, 0}  // k-axis
        };

        JSONObject imuData = new JSONObject();
        try {
            imuData.put("posX", position[0]);
            imuData.put("posY", position[1]);
            imuData.put("posZ", position[2]);
            imuData.put("rotX", rotX);
            imuData.put("rotY", rotY);
            imuData.put("rotZ", rotZ);

            imuData.put("i_x", rotationMatrix[0][0]);
            imuData.put("i_y", rotationMatrix[1][0]);
            imuData.put("i_z", rotationMatrix[2][0]);

            imuData.put("j_x", rotationMatrix[0][1]);
            imuData.put("j_y", rotationMatrix[1][1]);
            imuData.put("j_z", rotationMatrix[2][1]);

            imuData.put("k_x", rotationMatrix[0][2]);
            imuData.put("k_y", rotationMatrix[1][2]);
            imuData.put("k_z", rotationMatrix[2][2]);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("IMU Data", imuData.toString());
        telemetry.update();

        client.sendData(imuData.toString());
    }
}
