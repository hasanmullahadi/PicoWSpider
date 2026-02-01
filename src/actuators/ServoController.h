#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo configuration constants
static const uint16_t SERVO_FREQ_HZ = 60;
static const uint16_t SERVO_MIN_PULSE = 150;
static const uint16_t SERVO_MAX_PULSE = 600;
static const uint16_t SERVO_ANGLE_MIN = 0;
static const uint16_t SERVO_ANGLE_MAX = 180;

class ServoController {
public:
    ServoController(uint8_t addr = 0x40);

    void begin();
    void moveServo(uint8_t servoNum, uint16_t angle);
    void moveLeft(uint8_t servoNum);
    void moveRight(uint8_t servoNum);

private:
    Adafruit_PWMServoDriver pwm;
};

#endif
