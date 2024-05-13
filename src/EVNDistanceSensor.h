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
        _sensor_started = false;
        _port = constrain(port, 1, 16);
        _timing_budget_ms = constrain(timing_budget_ms, 20, 200);
    };

    bool begin()
    {
        if (_port > 8)
            sensor.setBus(&Wire1);

        EVNAlpha::sharedPorts().begin();
        EVNAlpha::sharedPorts().setPort(_port);

        if (!sensor.init())
        {
            return false;
        }

        _sensor_started = true;

        setSignalRateLimit(0.25);
        setPulsePeriodPreRange(14);
        setPulsePeriodFinalRange(10);

        sensor.setTimeout(0);
        this->setTimingBudget(_timing_budget_ms);
        sensor.startContinuous();

        return true;
    };

    void setSignalRateLimit(float limit)
    {
        if (_sensor_started)
        {
            EVNAlpha::sharedPorts().setPort(_port);
            sensor.setSignalRateLimit(0.25);
        }
    };

    void setPulsePeriodPreRange(uint8_t period)
    {
        if (_sensor_started)
        {
            EVNAlpha::sharedPorts().setPort(_port);
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, period);
        }
    };

    void setPulsePeriodFinalRange(uint8_t period)
    {
        if (_sensor_started)
        {
            EVNAlpha::sharedPorts().setPort(_port);
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, period);
        }
    };

    void setTimingBudget(uint8_t timing_budget_ms)
    {
        if (_sensor_started)
        {
            EVNAlpha::sharedPorts().setPort(_port);
            _timing_budget_ms = constrain(timing_budget_ms, 20, 200);
            sensor.setMeasurementTimingBudget(_timing_budget_ms * 1000);
        }
    };

    uint8_t getTimingBudget()
    {
        return _timing_budget_ms;
    };

    uint16_t read(bool blocking = true)
    {
        if (_sensor_started)
        {
            EVNAlpha::sharedPorts().setPort(_port);

            if (blocking)
            {
                _reading = sensor.readRangeContinuousMillimeters();
            }
            else
            {
                uint16_t possible_reading = sensor.readRangeContinuousMillimetersNonBlocking();
                if (possible_reading != 65535)
                    _reading = possible_reading;
            }

            return _reading;
        }
        return 0;
    };

private:
    bool _sensor_started;
    VL53L0X sensor;
    uint16_t _reading;
    uint8_t _port;
    uint16_t _timing_budget_ms;
};

#endif