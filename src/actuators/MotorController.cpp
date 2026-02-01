#include "MotorController.h"

MotorController::MotorController(uint8_t pwmaPin, uint8_t ain1Pin, uint8_t ain2Pin,
                                 uint8_t pwmbPin, uint8_t bin1Pin, uint8_t bin2Pin,
                                 uint8_t stbyPin)
    : _pwmaPin(pwmaPin), _ain1Pin(ain1Pin), _ain2Pin(ain2Pin),
      _pwmbPin(pwmbPin), _bin1Pin(bin1Pin), _bin2Pin(bin2Pin),
      _stbyPin(stbyPin) {}

void MotorController::begin() {
    pinMode(_pwmaPin, OUTPUT);
    pinMode(_ain1Pin, OUTPUT);
    pinMode(_ain2Pin, OUTPUT);
    pinMode(_pwmbPin, OUTPUT);
    pinMode(_bin1Pin, OUTPUT);
    pinMode(_bin2Pin, OUTPUT);
    pinMode(_stbyPin, OUTPUT);
    wake();
}

void MotorController::motorAForward(uint8_t speed) {
    motorControl(_pwmaPin, _ain1Pin, _ain2Pin, speed, true);
}

void MotorController::motorABackward(uint8_t speed) {
    motorControl(_pwmaPin, _ain1Pin, _ain2Pin, speed, false);
}

void MotorController::motorAStop() {
    analogWrite(_pwmaPin, 0);
}

void MotorController::motorBForward(uint8_t speed) {
    motorControl(_pwmbPin, _bin1Pin, _bin2Pin, speed, true);
}

void MotorController::motorBBackward(uint8_t speed) {
    motorControl(_pwmbPin, _bin1Pin, _bin2Pin, speed, false);
}

void MotorController::motorBStop() {
    analogWrite(_pwmbPin, 0);
}

void MotorController::standby() {
    digitalWrite(_stbyPin, LOW);
}

void MotorController::wake() {
    digitalWrite(_stbyPin, HIGH);
}

void MotorController::softStartA(uint8_t targetSpeed, uint8_t incrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = 0;
    unsigned long currentMillis = millis();

    if (currentSpeed < targetSpeed && (currentMillis - previousMillis >= incrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed += SPEED_INCREMENT;
        motorAForward(currentSpeed);
    }
}

void MotorController::softStopA(uint8_t decrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = analogRead(_pwmaPin);
    unsigned long currentMillis = millis();

    if (currentSpeed > 0 && (currentMillis - previousMillis >= decrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed = (currentSpeed > SPEED_INCREMENT) ? currentSpeed - SPEED_INCREMENT : 0;
        motorAForward(currentSpeed);
    }
    if (currentSpeed == 0) {
        motorAStop();
    }
}

void MotorController::softStartB(uint8_t targetSpeed, uint8_t incrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = 0;
    unsigned long currentMillis = millis();

    if (currentSpeed < targetSpeed && (currentMillis - previousMillis >= incrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed += SPEED_INCREMENT;
        motorBForward(currentSpeed);
    }
}

void MotorController::softStopB(uint8_t decrementDelay) {
    static unsigned long previousMillis = 0;
    static uint8_t currentSpeed = analogRead(_pwmbPin);
    unsigned long currentMillis = millis();

    if (currentSpeed > 0 && (currentMillis - previousMillis >= decrementDelay)) {
        previousMillis = currentMillis;
        currentSpeed = (currentSpeed > SPEED_INCREMENT) ? currentSpeed - SPEED_INCREMENT : 0;
        motorBForward(currentSpeed);
    }
    if (currentSpeed == 0) {
        motorBStop();
    }
}

void MotorController::motorControl(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin,
                                   uint8_t speed, bool forward) {
    analogWrite(pwmPin, speed);
    if (forward) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
}
