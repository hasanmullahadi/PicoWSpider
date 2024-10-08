#include "html.h"

const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control</title>
    <link rel="stylesheet" href="/style.css">
</head>

<body>
    <h1>Robot Control Interface</h1>

    <!-- Section for DHT values -->
    <div class="dht-data">
        <h2>Temperature and Humidity</h2>
        <p id="temperature">Temperature: Loading...</p>
        <p id="humidity">Humidity: Loading...</p>
    </div>


    <!-- Buttons for Motor Control -->
    <div class="button-container">
        <button onclick="sendCommand('/move-forward')">Move Forward</button>
        <button onclick="sendCommand('/move-backward')">Move Backward</button>
        <button onclick="sendCommand('/spin')">Spin</button>
        <button onclick="sendCommand('/stop')">Stop</button>
    </div>

    <!-- Slider for Speed Control -->
    <div class="speed-control">
        <label for="speed">Motor Speed Control:</label>
        <input type="range" min="0" max="100" value="50" id="speed" oninput="updateSpeed(this.value)">
        <p id="speedValue">50%</p>
    </div>

    <!-- Section for Distance Monitoring -->
    <div class="distance-monitor">
        <h2>Distance from Object:</h2>
        <p id="distanceValue">Loading...</p>
    </div>

    <!-- Section for RGB Control -->
    <div class="rgb-control">
        <h2>RGB Control</h2>

        <!-- Color Picker -->
        <label for="colorPicker">Select Color:</label>
        <input type="color" id="colorPicker" value="#ff0000" oninput="updateRGB()">

        <!-- Index Selector -->
        <label for="ledIndex">Select LED:</label>
        <select id="ledIndex" onchange="updateRGB()">
            <option value="0">LED 1</option>
            <option value="1">LED 2</option>
            <option value="2">LED 3</option>
            <option value="3">LED 4</option>
            <option value="4">LED 5</option>
        </select>
    </div>

    <!-- Section for Accelerometer and Gyroscope Display -->
    <div class="sensor-data">
        <h2>Acceleration (X, Y, Z)</h2>
        <p id="accelX">X: Loading...</p>
        <p id="accelY">Y: Loading...</p>
        <p id="accelZ">Z: Loading...</p>

        <h2>Gyroscope (X, Y, Z)</h2>
        <p id="gyroX">X: Loading...</p>
        <p id="gyroY">Y: Loading...</p>
        <p id="gyroZ">Z: Loading...</p>
    </div>

    <!-- Section for Sensor Graphs -->
    <div class="sensor-graph">
        <h2>Acceleration & Orientation Graph</h2>
        <canvas id="sensorCanvas" width="500" height="300"></canvas>
    </div>

    <script>
        const canvas = document.getElementById('sensorCanvas');
        const ctx = canvas.getContext('2d');

        // Function to send motor control commands
        function sendCommand(command) {
            fetch(command).then(response => {
                if (response.ok) {
                    console.log('Command sent successfully');
                } else {
                    console.log('Error sending command');
                }
            });
        }

        // Function to update motor speed based on slider value
        function updateSpeed(value) {
            document.getElementById('speedValue').innerHTML = value + '%';  // Update the displayed speed percentage
            fetch('/set-speed?value=' + value);  // Send speed to server
        }

        // Function to update the distance reading from the sonar sensor
        function updateDistance() {
            fetch('/get-distance').then(response => response.text()).then(distance => {
                document.getElementById('distanceValue').innerHTML = distance + ' cm';  // Update the displayed distance
            });
        }

        // Function to update RGB LED color based on the selected color and LED index
        function updateRGB() {
            const colorPicker = document.getElementById('colorPicker');
            const ledIndex = document.getElementById('ledIndex').value;
            const color = colorPicker.value.substring(1);  // Remove the '#' from the color hex

            // Send the color and index to the server
            fetch(`/set-rgb?index=${ledIndex}&color=${color}`).then(response => {
                if (response.ok) {
                    console.log('RGB color updated successfully');
                } else {
                    console.log('Error updating RGB color');
                }
            });
        }

        // Function to fetch accelerometer and gyroscope data and update the graph and values
        function updateSensorData() {
            fetch('/get-acceleration').then(response => response.json()).then(accelData => {
                document.getElementById('accelX').innerHTML = "X: " + accelData.x.toFixed(2);
                document.getElementById('accelY').innerHTML = "Y: " + accelData.y.toFixed(2);
                document.getElementById('accelZ').innerHTML = "Z: " + accelData.z.toFixed(2);

                drawGraph(accelData.x, accelData.y, accelData.z);
            });

            fetch('/get-gyroscope').then(response => response.json()).then(gyroData => {
                document.getElementById('gyroX').innerHTML = "X: " + gyroData.x.toFixed(2);
                document.getElementById('gyroY').innerHTML = "Y: " + gyroData.y.toFixed(2);
                document.getElementById('gyroZ').innerHTML = "Z: " + gyroData.z.toFixed(2);
            });
        }

        // Function to draw the acceleration graph
        function drawGraph(x, y, z) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);  // Clear the canvas

            // Draw X, Y, and Z bars
            ctx.fillStyle = 'red';
            ctx.fillRect(50, 150 - x * 50, 50, x * 50);
            ctx.fillStyle = 'green';
            ctx.fillRect(150, 150 - y * 50, 50, y * 50);
            ctx.fillStyle = 'blue';
            ctx.fillRect(250, 150 - z * 50, 50, z * 50);

            // Labeling the axes
            ctx.fillStyle = 'black';
            ctx.fillText("X", 70, 170);
            ctx.fillText("Y", 170, 170);
            ctx.fillText("Z", 270, 170);
        }

        // Function to fetch DHT values and update the website
        function updateDHT() {
            fetch('/get-dht').then(response => response.json()).then(dhtData => {
                document.getElementById('temperature').innerHTML = "Temperature: " + dhtData.temperature.toFixed(2) + " °C";
                document.getElementById('humidity').innerHTML = "Humidity: " + dhtData.humidity.toFixed(2) + " %";
            });
        }


        // Update the sensor data every 2 seconds
        setInterval(updateSensorData, 250);
        // Update the distance value every 2 seconds
        setInterval(updateDistance, 250);
        // Update the DHT values every 2 seconds
        setInterval(updateDHT, 1000);

    </script>
</body>

</html>
)rawliteral";

