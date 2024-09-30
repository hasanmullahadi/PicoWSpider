#include "DHTSensor.h"

// Constructor
DHTSensor::DHTSensor(uint8_t pin, uint8_t type) : _pin(pin), _type(type), _dht(pin, type) {}

// Initialize the sensor
void DHTSensor::begin() {
    _dht.begin();
}

// Read temperature in Celsius
float DHTSensor::readTemperature() {
    return _dht.readTemperature();
}

// Read humidity
float DHTSensor::readHumidity() {
    return _dht.readHumidity();
}
