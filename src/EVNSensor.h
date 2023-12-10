#ifndef EVNSensor_h
#define EVNSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"

class EVNSensor {

    static const uint8_t COMMAND_BIT = 0x80;

public:
    EVNSensor(uint8_t port)
    {
        _port = constrain(port, 1, 16);
        if (_port <= 8)
            _wire = &Wire;
        else
            _wire = &Wire1;
    };

    void beginPortSelector()
    {
        EVNAlpha::sharedPorts().begin();
    };

    void write8(uint8_t addr, uint8_t reg, uint8_t value)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        EVNAlpha::sharedPorts().setPort(_port);
        _wire->beginTransmission(addr);
        _wire->write(COMMAND_BIT | reg);
        _wire->write(value);
        _wire->endTransmission();
        EVNAlpha::sharedPorts().setPort(prev_port);
    };

    uint8_t read8(uint8_t addr, uint8_t reg)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        EVNAlpha::sharedPorts().setPort(_port);
        _wire->beginTransmission(addr);
        _wire->write(COMMAND_BIT | reg);
        _wire->endTransmission();
        _wire->requestFrom(addr, (uint8_t)1);
        uint8_t out = _wire->read();
        EVNAlpha::sharedPorts().setPort(prev_port);
        return out;
    };
    uint16_t read16(uint8_t addr, uint8_t reg)
    {
        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        EVNAlpha::sharedPorts().setPort(_port);
        uint16_t high, low;
        _wire->beginTransmission(addr);
        _wire->write(COMMAND_BIT | reg);
        _wire->endTransmission();
        _wire->requestFrom(addr, (uint8_t)2);
        low = _wire->read();
        high = _wire->read();
        high <<= 8;
        high |= low;
        EVNAlpha::sharedPorts().setPort(prev_port);
        return high;
    };

private:
    uint8_t _port;
    TwoWire* _wire;
};

#endif