#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
public:
    // Constructor
    Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    // Initialize the sensor
    void begin();

    // Get distance in centimeters
    float getDistance();

private:
    uint8_t _trigPin;
    uint8_t _echoPin;
};

#endif
