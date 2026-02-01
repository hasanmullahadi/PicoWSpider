#ifndef DHTSENSOR_H
#define DHTSENSOR_H

#include <Arduino.h>
#include <DHT.h>

class DHTSensor {
public:
    DHTSensor(uint8_t pin, uint8_t type);

    void begin();
    float readTemperature();
    float readHumidity();

private:
    uint8_t _pin;
    uint8_t _type;
    DHT _dht;
};

#endif
