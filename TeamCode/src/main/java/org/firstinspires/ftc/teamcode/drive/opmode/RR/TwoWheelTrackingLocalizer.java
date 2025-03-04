package org.firstinspires.ftc.teamcode.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
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
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 7.87; // Y is the strafe direction

    public static double PERPENDICULAR_X = 4.33;
    public static double PERPENDICULAR_Y = -2.75;

    public static double X_MULTIPLIER = 0.932491482;
    public static double Y_MULTIPLIER = 1.020739502;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        readData();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }


    public void readData() {
        String jsonData = readJSONFromFile();
        if (jsonData != null) {
            try {
                // Parse the JSON string
                JSONObject config = new JSONObject(jsonData);

                setEncoderDirection(parallelEncoder,
                        config.optString("rightdeadWheelDir", "FORWARD"));
                setEncoderDirection(perpendicularEncoder,
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
