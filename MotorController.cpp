#include "MotorController.h"

// Constructor
MotorController::MotorController(uint8_t pwmaPin, uint8_t ain1Pin, uint8_t ain2Pin, uint8_t pwmbPin, uint8_t bin1Pin, uint8_t bin2Pin, uint8_t stbyPin)
    : _pwmaPin(pwmaPin), _ain1Pin(ain1Pin), _ain2Pin(ain2Pin), _pwmbPin(pwmbPin), _bin1Pin(bin1Pin), _bin2Pin(bin2Pin), _stbyPin(stbyPin) {}

// Initialize the motor driver
void MotorController::begin() {
    pinMode(_pwmaPin, OUTPUT);
    pinMode(_ain1Pin, OUTPUT);
    pinMode(_ain2Pin, OUTPUT);
    pinMode(_pwmbPin, OUTPUT);
    pinMode(_bin1Pin, OUTPUT);
    pinMode(_bin2Pin, OUTPUT);
    pinMode(_stbyPin, OUTPUT);
    wake();  // Wake up the motor driver
}

// Control Motor A forward
void MotorController::motorAForward(uint8_t speed) {
    motorControl(_pwmaPin, _ain1Pin, _ain2Pin, speed, true);
}

// Control Motor A backward
void MotorController::motorABackward(uint8_t speed) {
    motorControl(_pwmaPin, _ain1Pin, _ain2Pin, speed, false);
}

// Stop Motor A
void MotorController::motorAStop() {
    analogWrite(_pwmaPin, 0);
}

// Control Motor B forward
void MotorController::motorBForward(uint8_t speed) {
    motorControl(_pwmbPin, _bin1Pin, _bin2Pin, speed, true);
}

// Control Motor B backward
void MotorController::motorBBackward(uint8_t speed) {
    motorControl(_pwmbPin, _bin1Pin, _bin2Pin, speed, false);
}

// Stop Motor B
void MotorController::motorBStop() {
    analogWrite(_pwmbPin, 0);
}

// Standby mode
void MotorController::standby() {
    digitalWrite(_stbyPin, LOW);
}

// Wake up from standby mode
void MotorController::wake() {
    digitalWrite(_stbyPin, HIGH);
}

// Non-blocking soft start for Motor A
void MotorController::softStartA(uint8_t targetSpeed, uint8_t incrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = 0;

    unsigned long currentMillis = millis();
    
    if (currentSpeed < targetSpeed && (currentMillis - previousMillis >= incrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed += 5;
        motorAForward(currentSpeed);
    }
}

// Non-blocking soft stop for Motor A
void MotorController::softStopA(uint8_t decrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = analogRead(_pwmaPin);

    unsigned long currentMillis = millis();

    if (currentSpeed > 0 && (currentMillis - previousMillis >= decrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed -= 5;
        motorAForward(currentSpeed);
    }
    if (currentSpeed <= 0) {
        motorAStop();
    }
}

// Non-blocking soft start for Motor B
void MotorController::softStartB(uint8_t targetSpeed, uint8_t incrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = 0;

    unsigned long currentMillis = millis();

    if (currentSpeed < targetSpeed && (currentMillis - previousMillis >= incrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed += 5;
        motorBForward(currentSpeed);
    }
}

// Non-blocking soft stop for Motor B
void MotorController::softStopB(uint8_t decrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = analogRead(_pwmbPin);

    unsigned long currentMillis = millis();

    if (currentSpeed > 0 && (currentMillis - previousMillis >= decrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed -= 5;
        motorBForward(currentSpeed);
    }
    if (currentSpeed <= 0) {
        motorBStop();
    }
}

// Private method to control motors
void MotorController::motorControl(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t speed, bool forward) {
    analogWrite(pwmPin, speed);
    if (forward) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
}
