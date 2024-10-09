package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class sensorSystem {

    public boolean weGotPixels(TouchSensor sensor) {
        return sensor.isPressed();
    }
}
