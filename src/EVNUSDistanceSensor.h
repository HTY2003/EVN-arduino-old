#ifndef EVNUSDistanceSensor_h
#define EVNUSDistanceSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"
#include "EVNSensor.h"

class EVNUSDistanceSensor : private EVNSensor {
public:
    static const uint8_t I2C_ADDR = 0x57;
    static const uint32_t SENSOR_TIMEOUT_US = 105000;
    static const uint32_t SENSOR_TIMEOUT_READING_UM = 7958040;
    static const uint32_t SENSOR_TIMEOUT_READING_UM = 7958040;

    EVNUSDistanceSensor(uint8_t port) : EVNSensor(port)
    {
        _timeout_reading = SENSOR_TIMEOUT_READING_UM;
        _trigger_sent = false;
        _sensor_started = false;
        _distance_um = 0;
    };

    bool begin()
    {
        if (!_sensor_started)
        {
            EVNAlpha::sharedPorts().begin();

            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
            EVNAlpha::sharedPorts().setPort(_port);

            _sensor_started = true;
            delayMicroseconds(SENSOR_TIMEOUT_US);

            if (this->trigger())
            {
                delay(1);
                _distance_um = this->readRaw();
            }
            else
            {
                _sensor_started = false;
            }

            EVNAlpha::sharedPorts().setPort(prev_port);

            return _sensor_started;
        }
        return true;
    };

    void setTimeoutReading(uint32_t timeout_reading)
    {
        _timeout_reading = timeout_reading;
    };

    uint32_t getTimeoutReading()
    {
        return _timeout_reading;
    };

    bool trigger()
    {
        if (_sensor_started)
        {
            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
            EVNAlpha::sharedPorts().setPort(_port);

            _wire->beginTransmission(I2C_ADDR);
            _wire->write(0x01);
            bool success = _wire->endTransmission() == 0;

            if (success)
            {
                _last_trigger_time_us = micros();
                _trigger_sent = true;
            }

            EVNAlpha::sharedPorts().setPort(prev_port);
            return success;
        }
        return false;
    };

    uint32_t read(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (blocking)
            {
                bool trigger_success = trigger();
                delay(1);
                uint32_t reading = this->readRaw();

                if (reading == SENSOR_TIMEOUT_READING_UM)
                    _distance_um = _timeout_reading;
                else if (reading != 0)
                    _distance_um = reading;
            }

            else
            {
                if (_trigger_sent)
                {
                    if (micros() - _last_trigger_time_us >= SENSOR_TIMEOUT_US)
                    {
                        uint32_t reading = this->readRaw();
                        trigger();

                        if (reading == SENSOR_TIMEOUT_READING_UM)
                            _distance_um = _timeout_reading;
                        else if (reading != 0)
                            _distance_um = reading;
                    }
                }
                else
                {
                    trigger();
                }
            }
        }
        return _distance_um;
    }

private:
    uint32_t readRaw()
    {
        uint32_t reading = 0;
        if (_trigger_sent)
        {
            uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
            EVNAlpha::sharedPorts().setPort(_port);

            //requestFrom() is blocking until a reading is available
            _wire->requestFrom(I2C_ADDR, 3);

            if (_wire->available() == 3)
            {
                reading = _wire->read() << 16 | _wire->read() << 8 | _wire->read();
                //if sensor returns 0, most likely a trigger has not been sent
                if (reading != 0)
                    _trigger_sent = false;
            }
            //if wire is not available, sensor is mid-trigger

            EVNAlpha::sharedPorts().setPort(prev_port);
        }
        return reading;
    };

private:
    uint64_t _last_trigger_time_us;
    uint32_t _distance_um, _timeout_reading;
    bool _trigger_sent;
    bool _sensor_started;
};

#endif