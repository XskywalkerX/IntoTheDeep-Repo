package org.firstinspires.ftc.teamcode.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.458100445871537; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.72440944882; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.932491482; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.020739502; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions,
                                          List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight")); /* esse aqui é o left encoder */
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));/* esse aqui é o right encoder */
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight")); /* esse aqui é o heading encoder */

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        readData();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }


    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }


    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    public Encoder getFrontEncoder() {
        return frontEncoder;
    }


    public void readData() {
        String jsonData = readJSONFromFile();
        if (jsonData != null) {
            try {
                // Parse the JSON string
                JSONObject config = new JSONObject(jsonData);

                setEncoderDirection(getLeftEncoder(),
                        config.optString("leftdeadWheelDir", "FORWARD"));
                setEncoderDirection(getRightEncoder(),
                        config.optString("rightdeadWheelDir", "FORWARD"));
                setEncoderDirection(getFrontEncoder(),
                        config.optString("frontdeadWheelDir", "FORWARD"));

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

    private void setEncoderDirection(Encoder encoder, String direction) {
        if (direction.equalsIgnoreCase("FORWARD")) {
            encoder.setDirection(Encoder.Direction.FORWARD);
        } else if (direction.equalsIgnoreCase("REVERSE")) {
            encoder.setDirection(Encoder.Direction.REVERSE);
        } else {
            encoder.setDirection(Encoder.Direction.FORWARD);
        }
    }
}