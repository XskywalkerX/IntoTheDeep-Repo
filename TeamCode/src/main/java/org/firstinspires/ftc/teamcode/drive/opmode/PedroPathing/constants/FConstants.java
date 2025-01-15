package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;

@Config
public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.TWO_WHEEL;

        FollowerConstants.leftFrontMotorName = "frontLeft";
        FollowerConstants.leftRearMotorName = "backLeft";
        FollowerConstants.rightFrontMotorName = "frontRight";
        FollowerConstants.rightRearMotorName = "backRight";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        readData();

        FollowerConstants.mass = 8;

        FollowerConstants.xMovement = 73.44492636070622;
        FollowerConstants.yMovement = 56.3945193948537;

        FollowerConstants.forwardZeroPowerAcceleration = -47.609392697762324;
        FollowerConstants.lateralZeroPowerAcceleration = -84.96820964709732;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.5,0,0,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.1,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.006,0,0,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0009;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }

    public static void readData() {
        String jsonData = readJSONFromFile();
        if (jsonData != null) {
            try {
                // Parse the JSON string
                JSONObject config = new JSONObject(jsonData);

                FollowerConstants.leftFrontMotorDirection = setMotorDirection(config.optString("frontLeftDir", "REVERSE"));
                FollowerConstants.rightFrontMotorDirection = setMotorDirection(config.optString("frontRightDir", "REVERSE"));
                FollowerConstants.leftRearMotorDirection = setMotorDirection(config.optString("backLeftDir", "REVERSE"));
                FollowerConstants.rightRearMotorDirection = setMotorDirection(config.optString("backRightDir", "FORWARD"));

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


    private static DcMotorSimple.Direction setMotorDirection(String direction) {
        if (direction.equalsIgnoreCase("FORWARD")) {
            return DcMotorSimple.Direction.FORWARD;
        } else if (direction.equalsIgnoreCase("REVERSE")) {
            return DcMotorSimple.Direction.REVERSE;
        }
        return DcMotorSimple.Direction.FORWARD;
    }
}
