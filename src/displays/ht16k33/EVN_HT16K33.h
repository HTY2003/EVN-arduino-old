#ifndef EVN_HT16K33_h
#define EVN_HT16K33_h

#include <Arduino.h>
#include <Wire.h>
#include "../../EVNAlpha.h"
#include "../../helper/EVNI2CDevice.h"

class EVN_HT16K33 : private EVNI2CDevice {

    friend class EVNMatrixLED;
    friend class EVNSevenSegmentLED;

public:
    static const uint8_t I2C_ADDR = 0x71;

    enum reg : uint8_t
    {
        DATA = 0x00,
    };

    enum system_setup : uint8_t
    {
        STANDBY = 0b00100000,
        NORMAL = 0b00100001
    };

    enum mode : uint8_t
    {
        OFF = 0b10000000,
        ON = 0b10000001,
        BLINK_HZ_2 = 0b10000011,
        BLINK_HZ_1 = 0b10000101,
        BLINK_HZ_0_5 = 0b10000111,
    };

    enum ris : uint8_t
    {
        OUTPUT = 0b10100000
    };

    EVN_HT16K33(uint8_t port) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        if (_wire->endTransmission())
            return false;

        _sensor_started = true;

        write8noReg((uint8_t)system_setup::NORMAL);
        write8noReg((uint8_t)ris::OUTPUT);
        setDisplayMode(mode::ON);
        setBrightness(16);
        clearAll();

        return _sensor_started;
    };

    void setDisplayMode(mode mode)
    {
        if (_sensor_started)
        {
            write8noReg((uint8_t)mode);
        }
    };

    void setBrightness(uint8_t brightness)
    {
        if (_sensor_started)
        {
            uint8_t brightnessc = constrain(brightness, 1, 16);

            write8noReg((brightness - 1) | 0xE0);
        }
    };

    void writeRaw(uint8_t led, bool on, bool show = true)
    {
        if (_sensor_started)
        {
            if (led > 127) return;

            uint8_t index = led / 8;
            uint8_t sub_index = led % 8;

            if (on)
            {
                _buffer[index] |= (on << sub_index);
            }
            else
            {
                uint8_t mask = 0xFF;
                mask ^= 0b1 << sub_index;
                _buffer[index] &= mask;
            }

            if (show) update();
        }
    };

    void clearAll(bool show = true)
    {
        if (_sensor_started)
        {
            memset(_buffer, 0, sizeof(_buffer));
            if (show) this->update();
        }
    };

    void writeAll(bool show = true)
    {
        if (_sensor_started)
        {
            memset(_buffer, 255, sizeof(_buffer));
            if (show) this->update();
        }
    };

    void update()
    {
        if (_sensor_started)
        {
            writeBuffer((uint8_t)reg::DATA, 16, _buffer);
        }
    };

private:

    void write8noReg(uint8_t value)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        _wire->write(value);
        _wire->endTransmission();
    };

    uint8_t _buffer[16];
};

#endif