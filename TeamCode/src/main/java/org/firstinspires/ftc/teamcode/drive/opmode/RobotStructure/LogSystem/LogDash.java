package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;
public class LogDash extends NanoHTTPD {
    public LogDash() throws IOException {
        super(17801);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        System.out.println("\nRunning! Point your browsers to http://192.168.43.1:17801/ \n");
    }

    @Override
    public Response serve(IHTTPSession session) {
        String msg = "<html><body><h1>BTS</h1>\n";
        return newFixedLengthResponse(msg + "</body></html>\n");
    }
}