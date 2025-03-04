package org.firstinspires.ftc.teamcode.PedroPathing.constants;

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
        TwoWheelConstants.strafeTicksToInches = 0.0030606158957683293;
        TwoWheelConstants.forwardY = 1;
        TwoWheelConstants.strafeX = -2.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "backLeft";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "backRight";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}




