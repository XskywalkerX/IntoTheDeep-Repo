package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0030606158957683293;
        TwoWheelConstants.strafeTicksToInches = 0.002995663228458976;
        TwoWheelConstants.forwardY = -6.729050223;
        TwoWheelConstants.strafeX = 2.72440944882;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "backLeft";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "frontRight";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;

        readData();

        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    public static void readData() {
        String jsonData = readJSONFromFile();
        if (jsonData != null) {
            try {
                // Parse the JSON string
                JSONObject config = new JSONObject(jsonData);

                if(config.optString("rightdeadWheelDir", "FORWARD").equalsIgnoreCase("FORWARD")) {
                    TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
                } else {
                    TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
                }

                if(config.optString("frontdeadWheelDir", "FORWARD").equalsIgnoreCase("FORWARD")) {
                    TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
                } else {
                    TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
                }

                // Example: Print keys from the JSON to telemetry
                Iterator<String> keys = config.keys();
                while (keys.hasNext()) {
                    String key = keys.next();
                }
            } catch (Exception e) {
                ;
                e.printStackTrace();
            }
        }
    }

    private static String readJSONFromFile() {
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
}




