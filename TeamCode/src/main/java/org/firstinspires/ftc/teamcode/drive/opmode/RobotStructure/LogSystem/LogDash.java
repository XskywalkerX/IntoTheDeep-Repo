package org.firstinspires.ftc.teamcode.RobotStructure.LogSystem;

import fi.iki.elonen.NanoHTTPD;

public class LogDash extends NanoHTTPD {
    private String batteryVoltage = "0.0";  // Store battery voltage data
    private String temp = "0.0";

    public LogDash() throws Exception {
        super(17801);  // Set the port to 17801
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        System.out.println("Web server started. Point your browsers to http://192.168.43.1:17801/ \n");
    }

    @Override
    public Response serve(IHTTPSession session) {
        // Check if the user is requesting the /data page
        if (session.getUri().equals("/data")) {
            return serveDataPage();
        }

        // Default response for other pages
        return newFixedLengthResponse("<html><body><h1>BTS</h1></body></html>");
    }

    private Response serveDataPage() {
        String htmlResponse = "<html>\n" +
                "<head>\n" +
                "    <style>\n" +
                "        body {\n" +
                "            background-color: black;\n" +
                "            color: white;\n" +
                "            font-family: 'Arial', sans-serif;\n" +
                "            margin: 0;\n" +
                "            padding: 0;\n" +
                "            display: flex;\n" +
                "            flex-direction: column;\n" +
                "            align-items: center;\n" +
                "        }\n" +
                "\n" +
                "        h1 {\n" +
                "            font-family: 'Arial', sans-serif;\n" +
                "            font-size: 3em;\n" +
                "            margin-top: 20px;\n" +
                "            margin-bottom: 40px;\n" +
                "            text-align: center;\n" +
                "            color: white;\n" +
                "        }\n" +
                "\n" +
                "        .container {\n" +
                "            display: flex;\n" +
                "            flex-wrap: wrap; /* Allows wrapping of boxes to the next line if the screen is small */\n" +
                "            justify-content: space-around;\n" +
                "            gap: 20px; /* Adds space between the boxes */\n" +
                "        }\n" +
                "\n" +
                "        .box {\n" +
                "            background-color: rgba(0, 0, 0, 0.8);\n" +
                "            padding: 20px;\n" +
                "            width: 300px;\n" +
                "            border: 2px solid white; /* Solid white border */\n" +
                "        }\n" +
                "\n" +
                "        .box h2 {\n" +
                "            margin: 0;\n" +
                "            font-size: 1.5em;\n" +
                "            color: white;\n" +
                "        }\n" +
                "\n" +
                "        .box p {\n" +
                "            font-size: 1.2em;\n" +
                "            color: white;\n" +
                "            margin-top: 10px;\n" +
                "        }\n" +
                "\n" +
                "        #log {\n" +
                "            color: #ffeb3b;\n" +
                "            font-size: 1.2em;\n" +
                "        }\n" +
                "\n" +
                "        /* Nested Box for Battery Voltage */\n" +
                "        .voltage-box {\n" +
                "            padding: 10px;\n" +
                "            font-size: 1.5em;\n" +
                "            text-align: center;\n" +
                "            margin-top: 20px;\n" +
                "            color: white;\n" +
                "        }\n" +
                "    </style>\n" +
                "</head>\n" +
                "<body>\n" +
                "    <h1>BOT Status</h1>\n" +
                "\n" +
                "    <!-- Flex container to hold boxes side by side -->\n" +
                "    <div class=\"container\">\n" +
                "        <!-- Control Status Box -->\n" +
                "        <div class=\"box\" id=\"control-status-box\">\n" +
                "            <h2>Control Status</h2>\n" +
                "            <!-- Nested Battery Voltage Box -->\n" +
                "            <div id=\"voltage-box\" class=\"voltage-box\">\n" +
                "                Waiting for data...\n" +
                "            </div>\n" +
                "        </div>\n" +
                "\n" +
                "        <!-- Robot Pose Box -->\n" +
                "        <div class=\"box\" id=\"robot-pose-box\">\n" +
                "            <h2>Robot Pose</h2>\n" +
                "            <p id=\"robot-pose\">Waiting for data...</p>\n" +
                "        </div>\n" +
                "\n" +
                "        <!-- Motors Status Box -->\n" +
                "        <div class=\"box\" id=\"motors-status-box\">\n" +
                "            <h2>Motors Status</h2>\n" +
                "            <p id=\"motors-status\">Waiting for data...</p>\n" +
                "        </div>\n" +
                "\n" +
                "        <!-- Camera Box -->\n" +
                "        <div class=\"box\" id=\"camera-box\">\n" +
                "            <h2>Camera</h2>\n" +
                "            <p id=\"camera-status\">Waiting for data...</p>\n" +
                "        </div>\n" +
                "    </div>\n" +
                "\n" +
                "    <div id=\"log\"></div>\n" +
                "\n" +
                "    <script type=\"text/javascript\">\n" +
                "        const socket = new WebSocket('ws://192.168.43.1:17802');  // WebSocket server address\n" +
                "\n" +
                "        socket.onopen = function() {\n" +
                "            document.getElementById('log').innerText = \"WebSocket connection established.\";\n" +
                "            console.log(\"WebSocket connection established.\");\n" +
                "        };\n" +
                "\n" +
                "        socket.onmessage = function(event) {\n" +
                "            const batteryVoltage = parseFloat(event.data).toFixed(2);  // Limit to two decimal places\n" +
                "            console.log(\"Received battery voltage: \" + batteryVoltage);  // Log received data\n" +
                "\n" +
                "            // Update the voltage display\n" +
                "            const voltageBox = document.getElementById('voltage-box');\n" +
                "            voltageBox.innerText = batteryVoltage + \" V\";\n" +
                "\n" +
                "            // Update the background color of the voltage box based on voltage\n" +
                "            if (batteryVoltage >= 12) {\n" +
                "                voltageBox.style.backgroundColor = 'green';  // Green for good battery\n" +
                "            } else if (batteryVoltage >= 11 && batteryVoltage < 12) {\n" +
                "                voltageBox.style.backgroundColor = 'orange';  // Orange for mid battery\n" +
                "            } else {\n" +
                "                voltageBox.style.backgroundColor = 'red';  // Red for low battery\n" +
                "            }\n" +
                "\n" +
                "            // Example for robot pose update (You can add other data types similarly)\n" +
                "            // document.getElementById('robot-pose').innerText = \"Robot Pose: \" + robotPoseData;\n" +
                "        };\n" +
                "\n" +
                "        socket.onclose = function() {\n" +
                "            document.getElementById('log').innerText = \"WebSocket connection closed.\";\n" +
                "            console.log(\"WebSocket connection closed.\");\n" +
                "        };\n" +
                "\n" +
                "        socket.onerror = function(error) {\n" +
                "            document.getElementById('log').innerText = \"WebSocket Error: \" + error;\n" +
                "            console.error(\"WebSocket Error: \" + error);\n" +
                "        };\n" +
                "    </script>\n" +
                "</body>\n" +
                "</html>\n";
        return newFixedLengthResponse(htmlResponse);
    }

    // Method to update the battery voltage
    public void updateBatteryVoltage(String voltage) {
        this.batteryVoltage = voltage;
    }
    public void updateTemperature(String temp) {
        this.temp = temp;
    }
}
