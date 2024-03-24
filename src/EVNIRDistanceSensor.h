#ifndef EVNIRDistanceSensor_h
#define EVNIRDistanceSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"
#include "EVNSensor.h"
#include "vl53l0x-arduino/VL53L0X.h"

class EVNIRDistanceSensor {
public:
    EVNIRDistanceSensor(uint8_t port, uint8_t timing_budget_ms = 33) : sensor()
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

        uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
        EVNAlpha::sharedPorts().setPort(_port);

        if (!sensor.init())
        {
            EVNAlpha::sharedPorts().setPort(prev_port);
            return false;
        }

        _sensor_started = true;

        sensor.setSignalRateLimit(0.25);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

        sensor.setTimeout(0);
        this->setTimingBudget(_timing_budget_ms);
        sensor.startContinuous();

        EVNAlpha::sharedPorts().setPort(prev_port);
        return true;
    };

    void setTimingBudget(uint8_t timing_budget_ms)
    {
        if (_sensor_started)
        {
            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
            EVNAlpha::sharedPorts().setPort(_port);

            _timing_budget_ms = constrain(timing_budget_ms, 20, 200);
            sensor.setMeasurementTimingBudget(_timing_budget_ms * 1000);

            EVNAlpha::sharedPorts().setPort(prev_port);
        }
    }

    uint8_t getTimingBudget()
    {
        return _timing_budget_ms;
    }

    uint16_t read(bool blocking = true)
    {
        if (_sensor_started)
        {
            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
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
            EVNAlpha::sharedPorts().setPort(prev_port);

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