#ifndef EVNSensor_h
#define EVNSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"

class EVNSensor {

public:
    EVNSensor(uint8_t port)
    {
        _port = constrain(port, 1, 16);
        if (_port <= 8)
            _wire = &Wire;
        else
            _wire = &Wire1;
    };

    void write8(uint8_t addr, uint8_t reg, uint8_t value, bool control_port = true)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        if (control_port) { EVNAlpha::sharedPorts().setPort(_port); }
        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->write(value);
        _wire->endTransmission();
        if (control_port) EVNAlpha::sharedPorts().setPort(prev_port);
    };

    uint8_t read8(uint8_t addr, uint8_t reg, bool control_port = true)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        if (control_port) { EVNAlpha::sharedPorts().setPort(_port); }

        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->endTransmission();
        _wire->requestFrom(addr, (uint8_t)1);
        uint8_t out = _wire->read();
        if (control_port) EVNAlpha::sharedPorts().setPort(prev_port);
        return out;
    };
    uint16_t read16(uint8_t addr, uint8_t reg, bool lsb_start = true, bool control_port = true)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        if (control_port) { EVNAlpha::sharedPorts().setPort(_port); }

        uint16_t high, low;
        _wire->beginTransmission(addr);
        _wire->write(reg);
        _wire->endTransmission();
        _wire->requestFrom(addr, (uint8_t)2);

        if (lsb_start)
        {
            low = _wire->read();
            high = _wire->read();
        }
        else
        {
            high = _wire->read();
            low = _wire->read();
        }
        high <<= 8;
        high |= low;
        return high;
        if (control_port) EVNAlpha::sharedPorts().setPort(prev_port);
    };

protected:
    uint8_t _port;
    TwoWire* _wire;
};

#endif