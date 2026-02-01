#include "ServoController.h"

ServoController::ServoController(uint8_t addr) : pwm(Adafruit_PWMServoDriver(addr)) {}

void ServoController::begin() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ_HZ);
}

void ServoController::moveServo(uint8_t servoNum, uint16_t angle) {
    uint16_t clampedAngle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    uint16_t pulseLength = map(clampedAngle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX,
                               SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(servoNum, 0, pulseLength);
}

void ServoController::moveLeft(uint8_t servoNum) {
    moveServo(servoNum, SERVO_ANGLE_MIN);
}

void ServoController::moveRight(uint8_t servoNum) {
    moveServo(servoNum, SERVO_ANGLE_MAX);
}
