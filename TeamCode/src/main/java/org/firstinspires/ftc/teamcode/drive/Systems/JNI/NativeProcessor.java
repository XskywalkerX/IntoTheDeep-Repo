package org.firstinspires.ftc.teamcode.drive.Systems.JNI;

public class NativeProcessor {

    static {
        System.loadLibrary("NativeProcessor");
    }

    public static native void processFrame(long matAddr);
}
