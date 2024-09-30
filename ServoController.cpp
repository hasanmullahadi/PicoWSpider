#include "ServoController.h"

// Constructor
ServoController::ServoController(uint8_t addr) : pwm(Adafruit_PWMServoDriver(addr)) {}

// Initialize the PWM driver
void ServoController::begin() {
    pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

// Move servo to a specific angle
void ServoController::moveServo(uint8_t servoNum, uint16_t angle) {
    uint16_t pulseLength = map(angle, 0, 180, servoMin, servoMax);
    pwm.setPWM(servoNum, 0, pulseLength);
}

// Move servo left
void ServoController::moveLeft(uint8_t servoNum) {
    moveServo(servoNum, 0);  // Move to 0 degrees
}

// Move servo right
void ServoController::moveRight(uint8_t servoNum) {
    moveServo(servoNum, 180);  // Move to 180 degrees
}
