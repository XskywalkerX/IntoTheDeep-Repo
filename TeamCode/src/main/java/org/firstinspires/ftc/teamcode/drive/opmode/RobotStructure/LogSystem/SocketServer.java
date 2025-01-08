package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SocketServer extends WebSocketServer {
    private static List<WebSocket> clients = new ArrayList<>();  // Keep track of connected clients
    private static Map<WebSocket, String> lastMessages = new HashMap<>();  // Store last message for each client

    public SocketServer(int port) {
        super(new InetSocketAddress("192.168.43.1", port));  // Server address and port
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        clients.add(conn);  // Add new client to the list
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        clients.remove(conn);  // Remove client from the list when closed
        lastMessages.remove(conn);
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        // Handle messages from clients (if necessary)
        lastMessages.put(conn, message);

        saveJSONToFile(message);
    }

    private void saveJSONToFile(String jsonData) {
        try {
            // Create or overwrite the file at the desired location
            File file = new File("/sdcard/TAMOPORCA/config.json");  // File path on Control Hub

            // Use FileWriter to write the data to the file
            FileWriter fileWriter = new FileWriter(file, false);  // false to overwrite the file
            fileWriter.write(jsonData);
            fileWriter.close();

            System.out.println("JSON saved to " + file.getAbsolutePath());
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error saving JSON to file");
        }
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
