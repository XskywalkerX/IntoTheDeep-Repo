package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.StereoVision;

public class StereoVision {
    static {
        System.loadLibrary("stereo_video_processing"); // Load the DLL (without .dll extension)
    }

    // Native method to process stereo video
    public native void processStereoVideo(int leftCamIndex, int rightCamIndex);

    public static void main(String[] args) {
        StereoVision processor = new StereoVision();

        // Replace 0 and 2 with the correct camera indices for your left and right cameras
        processor.processStereoVideo(0, 2);
    }
}
