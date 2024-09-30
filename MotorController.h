#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    // Constructor
    MotorController(uint8_t pwmaPin, uint8_t ain1Pin, uint8_t ain2Pin, uint8_t pwmbPin, uint8_t bin1Pin, uint8_t bin2Pin, uint8_t stbyPin);

    // Initialize the motor driver
    void begin();

    // Control Motor A
    void motorAForward(uint8_t targetSpeed);
    void motorABackward(uint8_t targetSpeed);
    void motorAStop();

    // Control Motor B
    void motorBForward(uint8_t targetSpeed);
    void motorBBackward(uint8_t targetSpeed);
    void motorBStop();

    // Standby Mode
    void standby();
    void wake();

    // Soft Start and Stop for Motor A
    void softStartA(uint8_t targetSpeed, uint8_t incrementDelay = 10);
    void softStopA(uint8_t decrementDelay = 10);

    // Soft Start and Stop for Motor B
    void softStartB(uint8_t targetSpeed, uint8_t incrementDelay = 10);
    void softStopB(uint8_t decrementDelay = 10);

private:
    uint8_t _pwmaPin, _ain1Pin, _ain2Pin, _pwmbPin, _bin1Pin, _bin2Pin, _stbyPin;

    void motorControl(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t speed, bool forward);
};

#endif
