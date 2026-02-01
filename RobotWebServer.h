#ifndef ROBOTWEBSERVER_H
#define ROBOTWEBSERVER_H

#include <WiFi.h>
#include <WebServer.h>
#include "html.h"
#include "css.h"

static const int RGB_LED_COUNT = 1;

class RobotWebServer {
public:
    RobotWebServer(const char *ssid = "PicoW",
                   const char *password = "12345",
                   int port = 80,
                   bool ap_mode = false);

    void begin();
    void handleClient();

    // Motor control callbacks
    void setMoveForwardCallback(void (*func)());
    void setMoveBackwardCallback(void (*func)());
    void setStopCallback(void (*func)());
    void setSpinCallback(void (*func)());

    // Motor speed
    void setMotorSpeed(int speed);
    int getMotorSpeed();

    // Distance sensor
    float getDistance();
    void setDistance(float dis);

    // RGB LED control
    void setRGBValue(int index, uint8_t r, uint8_t g, uint8_t b);
    void getRGBValue(int index, uint8_t &r, uint8_t &g, uint8_t &b);

    // IMU data
    void setAcceleration(float x, float y, float z);
    void getAcceleration(float &x, float &y, float &z);
    void setGyroscope(float x, float y, float z);
    void getGyroscope(float &x, float &y, float &z);

    // Environment sensor
    void setDHTValues(float temperature, float humidity);
    void getDHTValues(float &temperature, float &humidity);

private:
    WebServer server;
    const char *ssid;
    const char *password;
    bool ap_mode;

    int motorSpeed;
    float distance;

    uint8_t rgbValues[RGB_LED_COUNT][3];

    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float dhtTemperature, dhtHumidity;

    void (*moveForwardCallback)();
    void (*moveBackwardCallback)();
    void (*stopCallback)();
    void (*spinCallback)();

    void setupAP();

    // Route handlers
    void handleRoot();
    void handleCSS();
    void handleMoveForward();
    void handleMoveBackward();
    void handleStop();
    void handleSpin();
    void handleSetSpeed();
    void handleGetSpeed();
    void handleGetDistance();
    void handleSetRGB();
    void handleGetRGB();
    void handleGetAcceleration();
    void handleGetGyroscope();
    void handleGetDHT();
};

#endif
