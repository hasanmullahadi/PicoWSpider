#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class ServoController {
public:
    // Constructor
    ServoController(uint8_t addr = 0x40);

    // Initialize the PWM driver
    void begin();

    // Move servo to a specific angle
    void moveServo(uint8_t servoNum, uint16_t angle);

    // Move servo left
    void moveLeft(uint8_t servoNum);

    // Move servo right
    void moveRight(uint8_t servoNum);

private:
    Adafruit_PWMServoDriver pwm;
    uint16_t servoMin = 150;  // Minimum pulse length count (adjust based on your servo)
    uint16_t servoMax = 600;  // Maximum pulse length count (adjust based on your servo)
};

#endif
