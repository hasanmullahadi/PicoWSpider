#include "DHTSensor.h"

DHTSensor::DHTSensor(uint8_t pin, uint8_t type)
    : _pin(pin), _type(type), _dht(pin, type) {}

void DHTSensor::begin() {
    _dht.begin();
}

float DHTSensor::readTemperature() {
    return _dht.readTemperature();
}

float DHTSensor::readHumidity() {
    return _dht.readHumidity();
}
