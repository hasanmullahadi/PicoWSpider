#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin)
    : _trigPin(trigPin), _echoPin(echoPin) {}

void Ultrasonic::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

float Ultrasonic::getDistance() {
    // Send 10us trigger pulse
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    long duration = pulseIn(_echoPin, HIGH);

    // Convert duration to distance (divide by 2 for round-trip)
    return (duration * SOUND_CM_PER_US) / 2.0;
}
