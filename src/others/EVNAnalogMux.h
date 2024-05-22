#ifndef EVNAnalogMux_h
#define EVNAnalogMux_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

class EVNAnalogMux : private EVNI2CDevice {
public:

    enum class reg : uint8_t
    {
        CONFIG = 0x01,
    };

    enum class gain : uint8_t
    {

    };

    enum class mode : uint8_t
    {

    };

    enum class data_rate : uint8_t
    {

    };

    static const uint8_t I2C_ADDR = 0x48;
    static const uint8_t ID = 0x60;

    //TODO: Differential Pin Reads as additional pins

    EVNAnalogMux(uint8_t port) : EVNI2CDevice(port)
    {
        //reset
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        uint8_t id = read8((uint8_t)reg::CONFIG);

        if (id != ID)
        {
            return _sensor_started;
        }

        _sensor_started = true;

        return _sensor_started;
    };

    void setDataRate()
    {

    };

    void requestPin()
    {

    };

    uint32_t read(uint8_t pin = 0, bool blocking = true)
    {
        uint8_t pinc = constrain(pin, 0, 3);

        if (_sensor_started)
        {

        }
        return 0;
    };

    void setContinuous(uint8_t pin = 0)
    {

    };

    void readContinuous(uint8_t pin = 0)
    {

    };

private:
    uint32_t _adc1, _adc2, _adc3, _adc4;
};

#endif