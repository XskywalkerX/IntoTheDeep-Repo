package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.List;

public class SocketServer extends WebSocketServer {
    private static List<WebSocket> clients = new ArrayList<>();  // Keep track of connected clients

    public SocketServer() {
        super(new InetSocketAddress("192.168.43.1", 17802));  // Server address and port
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        clients.add(conn);  // Add new client to the list
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        clients.remove(conn);  // Remove client from the list when closed
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        // Handle messages from clients (if necessary)
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {
        System.out.println("Socket Server started!");
    }

    // Send battery voltage to all connected clients
    public void sendBatteryVoltage(String voltage) {
        for (WebSocket client : clients) {
            client.send(voltage);  // Send battery voltage to all connected clients
        }
    }

    public void sendData(String data) {
        for (WebSocket ws : clients) {
            ws.send(data);
        }
    }
}
