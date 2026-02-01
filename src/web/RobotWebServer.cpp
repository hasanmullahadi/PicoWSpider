#include "RobotWebServer.h"

RobotWebServer::RobotWebServer(const char *ssid, const char *password, int port, bool ap_mode)
    : server(port), ssid(ssid), password(password), ap_mode(ap_mode),
      moveForwardCallback(nullptr), moveBackwardCallback(nullptr),
      stopCallback(nullptr), spinCallback(nullptr) {}

void RobotWebServer::setupAP() {
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.print("AP started with SSID: ");
    Serial.println(ssid);
}

void RobotWebServer::begin() {
    if (ap_mode) {
        setupAP();
    } else {
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.println("Connecting to WiFi...");
        }
        Serial.println("Connected to WiFi");
    }

    // Register routes
    server.on("/", std::bind(&RobotWebServer::handleRoot, this));
    server.on("/style.css", std::bind(&RobotWebServer::handleCSS, this));
    server.on("/move-forward", std::bind(&RobotWebServer::handleMoveForward, this));
    server.on("/move-backward", std::bind(&RobotWebServer::handleMoveBackward, this));
    server.on("/stop", std::bind(&RobotWebServer::handleStop, this));
    server.on("/spin", std::bind(&RobotWebServer::handleSpin, this));
    server.on("/set-speed", std::bind(&RobotWebServer::handleSetSpeed, this));
    server.on("/get-speed", std::bind(&RobotWebServer::handleGetSpeed, this));
    server.on("/get-distance", std::bind(&RobotWebServer::handleGetDistance, this));
    server.on("/set-rgb", std::bind(&RobotWebServer::handleSetRGB, this));
    server.on("/get-rgb", std::bind(&RobotWebServer::handleGetRGB, this));
    server.on("/get-acceleration", std::bind(&RobotWebServer::handleGetAcceleration, this));
    server.on("/get-gyroscope", std::bind(&RobotWebServer::handleGetGyroscope, this));
    server.on("/get-dht", std::bind(&RobotWebServer::handleGetDHT, this));

    server.begin();
    Serial.println("Server started");
}

void RobotWebServer::handleClient() {
    server.handleClient();
}

// Static content handlers
void RobotWebServer::handleRoot() {
    server.send(200, "text/html", index_html);
}

void RobotWebServer::handleCSS() {
    server.send(200, "text/css", style_css);
}

// Movement handlers
void RobotWebServer::handleMoveForward() {
    if (moveForwardCallback) moveForwardCallback();
    server.send(200, "text/plain", "Moving forward");
}

void RobotWebServer::handleMoveBackward() {
    if (moveBackwardCallback) moveBackwardCallback();
    server.send(200, "text/plain", "Moving backward");
}

void RobotWebServer::handleStop() {
    if (stopCallback) stopCallback();
    server.send(200, "text/plain", "Stopping");
}

void RobotWebServer::handleSpin() {
    if (spinCallback) spinCallback();
    server.send(200, "text/plain", "Spinning");
}

// Callback setters
void RobotWebServer::setMoveForwardCallback(void (*func)()) {
    moveForwardCallback = func;
}

void RobotWebServer::setMoveBackwardCallback(void (*func)()) {
    moveBackwardCallback = func;
}

void RobotWebServer::setStopCallback(void (*func)()) {
    stopCallback = func;
}

void RobotWebServer::setSpinCallback(void (*func)()) {
    spinCallback = func;
}

// Motor speed
void RobotWebServer::setMotorSpeed(int speed) {
    motorSpeed = speed;
    Serial.print("Motor speed set to: ");
    Serial.println(motorSpeed);
}

int RobotWebServer::getMotorSpeed() {
    return motorSpeed;
}

void RobotWebServer::handleSetSpeed() {
    if (server.hasArg("value")) {
        setMotorSpeed(server.arg("value").toInt());
        server.send(200, "text/plain", "Speed updated");
    } else {
        server.send(400, "text/plain", "Missing speed value");
    }
}

void RobotWebServer::handleGetSpeed() {
    server.send(200, "text/plain", String(getMotorSpeed()));
}

// Distance sensor
void RobotWebServer::setDistance(float dis) {
    distance = dis;
}

float RobotWebServer::getDistance() {
    return distance;
}

void RobotWebServer::handleGetDistance() {
    server.send(200, "text/plain", String(distance));
}

// RGB LED
void RobotWebServer::setRGBValue(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= 0 && index < RGB_LED_COUNT) {
        rgbValues[index][0] = r;
        rgbValues[index][1] = g;
        rgbValues[index][2] = b;
    }
}

void RobotWebServer::getRGBValue(int index, uint8_t &r, uint8_t &g, uint8_t &b) {
    if (index >= 0 && index < RGB_LED_COUNT) {
        r = rgbValues[index][0];
        g = rgbValues[index][1];
        b = rgbValues[index][2];
    }
}

void RobotWebServer::handleSetRGB() {
    if (server.hasArg("index") && server.hasArg("color")) {
        int index = server.arg("index").toInt();
        String hexColor = server.arg("color");
        long colorValue = strtol(hexColor.c_str(), nullptr, 16);

        uint8_t r = (colorValue >> 16) & 0xFF;
        uint8_t g = (colorValue >> 8) & 0xFF;
        uint8_t b = colorValue & 0xFF;

        setRGBValue(index, r, g, b);
        server.send(200, "text/plain", "RGB color updated");
    } else {
        server.send(400, "text/plain", "Missing index or color");
    }
}

void RobotWebServer::handleGetRGB() {
    if (server.hasArg("index")) {
        int index = server.arg("index").toInt();
        uint8_t r, g, b;
        getRGBValue(index, r, g, b);
        server.send(200, "text/plain", String(r) + "," + String(g) + "," + String(b));
    } else {
        server.send(400, "text/plain", "Missing index");
    }
}

// Acceleration (IMU)
void RobotWebServer::setAcceleration(float x, float y, float z) {
    accelX = x;
    accelY = y;
    accelZ = z;
}

void RobotWebServer::getAcceleration(float &x, float &y, float &z) {
    x = accelX;
    y = accelY;
    z = accelZ;
}

void RobotWebServer::handleGetAcceleration() {
    String json = "{\"x\":" + String(accelX) +
                  ",\"y\":" + String(accelY) +
                  ",\"z\":" + String(accelZ) + "}";
    server.send(200, "application/json", json);
}

// Gyroscope (IMU)
void RobotWebServer::setGyroscope(float x, float y, float z) {
    gyroX = x;
    gyroY = y;
    gyroZ = z;
}

void RobotWebServer::getGyroscope(float &x, float &y, float &z) {
    x = gyroX;
    y = gyroY;
    z = gyroZ;
}

void RobotWebServer::handleGetGyroscope() {
    String json = "{\"x\":" + String(gyroX) +
                  ",\"y\":" + String(gyroY) +
                  ",\"z\":" + String(gyroZ) + "}";
    server.send(200, "application/json", json);
}

// DHT sensor
void RobotWebServer::setDHTValues(float temperature, float humidity) {
    dhtTemperature = temperature;
    dhtHumidity = humidity;
}

void RobotWebServer::getDHTValues(float &temperature, float &humidity) {
    temperature = dhtTemperature;
    humidity = dhtHumidity;
}

void RobotWebServer::handleGetDHT() {
    String json = "{\"temperature\":" + String(dhtTemperature) +
                  ",\"humidity\":" + String(dhtHumidity) + "}";
    server.send(200, "application/json", json);
}
