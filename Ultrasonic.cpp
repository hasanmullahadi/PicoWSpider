#include "Ultrasonic.h"

// Constructor
Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin) 
  : _trigPin(trigPin), _echoPin(echoPin) {}

// Initialize the sensor
void Ultrasonic::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

// Get distance in centimeters
float Ultrasonic::getDistance() {
    // Send a 10 microsecond pulse to trigger the sensor
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Measure the duration of the echo pulse
    long duration = pulseIn(_echoPin, HIGH);

    // Calculate the distance (duration in microseconds to centimeters)
    float distance = (duration * 0.0343) / 2;

    return distance;
}
