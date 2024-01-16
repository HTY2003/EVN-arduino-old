#ifndef EVNDistanceSensor_h
#define EVNDistanceSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"
#include "EVNSensor.h"
#include "vl53l0x-arduino/VL53L0X.h"

class EVNDistanceSensor {
public:
    EVNDistanceSensor(uint8_t port, uint8_t timing_budget_ms = 33) : sensor()
    {
        _port = constrain(port, 1, 16);
        _timing_budget_ms = constrain(timing_budget_ms, 20, 200);
        _timing_budget_us = _timing_budget_ms * 1000;
    };

    bool begin()
    {
        if (_port > 8)
            sensor.setBus(&Wire1);

        EVNAlpha::sharedPorts().begin();

        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        EVNAlpha::sharedPorts().setPort(_port);

        sensor.setTimeout(0);
        if (!sensor.init())
        {
            EVNAlpha::sharedPorts().setPort(prev_port);
            return false;
        }

        sensor.setSignalRateLimit(0.15);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 12);

        sensor.startContinuous(_timing_budget_ms);
        EVNAlpha::sharedPorts().setPort(prev_port);
        return true;
    };

    void setTimingBudget(uint8_t timing_budget_ms)
    {
        _timing_budget_ms = constrain(timing_budget_ms, 20, 200);
        sensor.setMeasurementTimingBudget(timing_budget_ms * 1000);
        _timing_budget_us = timing_budget_ms * 1000;
    }

    uint8_t getTimingBudget()
    {
        return _timing_budget_ms;
    }

    uint16_t read()
    {
        if ((micros() - _last_reading_us) > _timing_budget_us)
        {
            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
            EVNAlpha::sharedPorts().setPort(_port);
            _reading = sensor.readRangeContinuousMillimeters();
            EVNAlpha::sharedPorts().setPort(prev_port);

            _last_reading_us = micros();
        }

        return _reading;
    };

private:
    VL53L0X sensor;
    uint32_t _timing_budget_us;
    uint8_t _timing_budget_ms;
    uint16_t _reading;
    uint32_t _last_reading_us;
    uint8_t _port;
};

#endif