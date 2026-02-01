#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// Speed of sound: 343 m/s = 0.0343 cm/microsecond
// Divide by 2 for round-trip measurement
static const float SOUND_CM_PER_US = 0.0343;

class Ultrasonic {
public:
    Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    void begin();
    float getDistance();

private:
    uint8_t _trigPin;
    uint8_t _echoPin;
};

#endif
