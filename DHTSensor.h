#ifndef DHTSENSOR_H
#define DHTSENSOR_H

#include <Arduino.h>
#include <DHT.h>

class DHTSensor {
public:
    // Constructor
    DHTSensor(uint8_t pin, uint8_t type);

    // Initialize the sensor
    void begin();

    // Read temperature in Celsius
    float readTemperature();

    // Read humidity
    float readHumidity();

private:
    uint8_t _pin;
    uint8_t _type;
    DHT _dht;
};

#endif
