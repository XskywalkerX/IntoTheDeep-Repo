package org.firstinspires.ftc.teamcode.RobotStructure.LogSystem;

import fi.iki.elonen.NanoHTTPD;

public class Dash extends NanoHTTPD {

    public Dash() throws Exception{
        super(17801);
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

        String html = "<!DOCTYPE html>\n" +
                "<html lang=\"en\">\n" +
                "<head>\n" +
                "    <meta charset=\"UTF-8\">\n" +
                "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n" +
                "    <title>Real-Time Axis Rendering</title>\n" +
                "    <style>\n" +
                "        canvas {\n" +
                "            border: 1px solid black;\n" +
                "            display: block;\n" +
                "            margin: 0 auto;\n" +
                "        }\n" +
                "    </style>\n" +
                "</head>\n" +
                "<body>\n" +
                "    <canvas id=\"axisCanvas\" width=\"500\" height=\"500\"></canvas>\n" +
                "    <script>\n" +
                "        // Set up canvas and context\n" +
                "        const canvas = document.getElementById('axisCanvas');\n" +
                "        const ctx = canvas.getContext('2d');\n" +
                "\n" +
                "        // Origin point for axes\n" +
                "        const origin = { x: 250, y: 250 };\n" +
                "\n" +
                "        // Scaling factor to make axes visible on the canvas\n" +
                "        const scale = 100;\n" +
                "\n" +
                "        // Initial axis positions\n" +
                "        const axisData = {\n" +
                "            x: { start: [origin.x, origin.y], end: [origin.x + scale, origin.y] }, // X-Axis\n" +
                "            y: { start: [origin.x, origin.y], end: [origin.x, origin.y - scale] }, // Y-Axis\n" +
                "            z: { start: [origin.x, origin.y], end: [origin.x + scale / 2, origin.y + scale / 2] } // Z-Axis\n" +
                "        };\n" +
                "\n" +
                "        // Flag to track if new data has been received\n" +
                "        let receivedNewData = false;\n" +
                "\n" +
                "        // Function to draw axes\n" +
                "        function drawAxes(data) {\n" +
                "            ctx.clearRect(0, 0, canvas.width, canvas.height);\n" +
                "\n" +
                "            // Draw X-Axis (Red)\n" +
                "            ctx.strokeStyle = 'red';\n" +
                "            ctx.beginPath();\n" +
                "            ctx.moveTo(data.x.start[0], data.x.start[1]);\n" +
                "            ctx.lineTo(data.x.end[0], data.x.end[1]);\n" +
                "            ctx.stroke();\n" +
                "\n" +
                "            // Draw Y-Axis (Green)\n" +
                "            ctx.strokeStyle = 'green';\n" +
                "            ctx.beginPath();\n" +
                "            ctx.moveTo(data.y.start[0], data.y.start[1]);\n" +
                "            ctx.lineTo(data.y.end[0], data.y.end[1]);\n" +
                "            ctx.stroke();\n" +
                "\n" +
                "            // Draw Z-Axis (Blue)\n" +
                "            ctx.strokeStyle = 'blue';\n" +
                "            ctx.beginPath();\n" +
                "            ctx.moveTo(data.z.start[0], data.z.start[1]);\n" +
                "            ctx.lineTo(data.z.end[0], data.z.end[1]);\n" +
                "            ctx.stroke();\n" +
                "        }\n" +
                "\n" +
                "        // Initialize WebSocket\n" +
                "        const socket = new WebSocket('ws://192.168.43.1:17802');\n" +
                "\n" +
                "        socket.onopen = () => {\n" +
                "            console.log('WebSocket connection established.');\n" +
                "        };\n" +
                "\n" +
                "        socket.onmessage = (event) => {\n" +
                "    try {\n" +
                "        const message = JSON.parse(event.data);\n" +
                "\n" +
                "        // Extract rotation angles from the message\n" +
                "        const { rotX, rotY, rotZ } = message;\n" +
                "\n" +
                "        // Convert rotation angles to radians\n" +
                "        const rotXRad = (rotX * Math.PI) / 180;\n" +
                "        const rotYRad = (rotY * Math.PI) / 180;\n" +
                "        const rotZRad = (rotZ * Math.PI) / 180;\n" +
                "\n" +
                "        // Define a scale factor for axis length\n" +
                "        const scale = 100;\n" +
                "\n" +
                "        // Calculate axis endpoints based on rotations\n" +
                "        axisData.x.end = [\n" +
                "            origin.x + scale * Math.cos(rotXRad),\n" +
                "            origin.y - scale * Math.sin(rotXRad) // Canvas Y-axis is inverted\n" +
                "        ];\n" +
                "\n" +
                "        axisData.y.end = [\n" +
                "            origin.x + scale * Math.cos(rotYRad),\n" +
                "            origin.y - scale * Math.sin(rotYRad)\n" +
                "        ];\n" +
                "\n" +
                "        axisData.z.end = [\n" +
                "            origin.x + scale * Math.cos(rotZRad),\n" +
                "            origin.y - scale * Math.sin(rotZRad)\n" +
                "        ];\n" +
                "\n" +
                "        // Redraw axes\n" +
                "        drawAxes(axisData);\n" +
                "    } catch (err) {\n" +
                "        console.error('Error parsing WebSocket message:', err);\n" +
                "    }\n" +
                "};\n" +
                "\n" +
                "\n" +
                "        socket.onclose = () => {\n" +
                "            console.log('WebSocket connection closed.');\n" +
                "        };\n" +
                "\n" +
                "        socket.onerror = (error) => {\n" +
                "            console.error('WebSocket error:', error);\n" +
                "        };\n" +
                "\n" +
                "        // Rendering loop to continuously update the canvas\n" +
                "        function updateCanvas() {\n" +
                "            if (receivedNewData) {\n" +
                "                drawAxes(axisData);\n" +
                "                receivedNewData = false; // Reset flag after rendering\n" +
                "            }\n" +
                "            requestAnimationFrame(updateCanvas); // Continuously call updateCanvas\n" +
                "        }\n" +
                "\n" +
                "        // Start rendering loop\n" +
                "        updateCanvas();\n" +
                "    </script>\n" +
                "</body>\n" +
                "</html>";

        return newFixedLengthResponse(html);
    }
}
