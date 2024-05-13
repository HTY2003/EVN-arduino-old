#ifndef EVNTouchArray_h
#define EVNTouchArray_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

struct evn_touch
{
    bool ch0;
    bool ch1;
    bool ch2;
    bool ch3;
    bool ch4;
    bool ch5;
    bool ch6;
    bool ch7;
    bool ch8;
    bool ch9;
    bool ch10;
    bool ch11;
    bool ch12;
};

class EVNTouchArray : private EVNI2CDevice {
public:
    static const uint8_t I2C_ADDR = 0x5A;
    static const uint8_t DEFAULT_CONFIG = 0x24;

    enum class reg : uint8_t
    {
        TOUCH_STATUS_0_7 = 0x00,
        TOUCH_STATUS_8_12 = 0x01,
        MHDR = 0x2B,
        NHDR = 0x2C,
        NCLR = 0x2D,
        FDLR = 0x2E,
        MHDF = 0x2F,
        NHDF = 0x30,
        NCLF = 0x31,
        FDLF = 0x32,
        NHDT = 0x33,
        NCLT = 0x34,
        FDLT = 0x35,
        TOUCHTH_0 = 0x41,
        RELEASETH_0 = 0x42,
        DEBOUNCE = 0x5B,
        AFE_CONFIG_1 = 0x5C,
        AFE_CONFIG_2 = 0x5D,
        ECR = 0x5E,
        SOFT_RESET = 0x80
    };

    enum class command : uint8_t
    {
        START = 0b11111100,
        STOP = 0b00000000
    };

    EVNTouchArray(uint8_t port, uint8_t touch_threshold = 12, uint8_t release_threshold = 6) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
        _touch_threshold = touch_threshold;
        _release_threshold = release_threshold;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        write8((uint8_t)reg::SOFT_RESET, 0x63);
        delay(10);
        write8((uint8_t)reg::ECR, (uint8_t)command::STOP);

        uint8_t id = read8((uint8_t)reg::AFE_CONFIG_2, false);

        if (id != DEFAULT_CONFIG)
        {
            return _sensor_started;
        }

        _sensor_started = true;

        write8((uint8_t)reg::ECR, (uint8_t)command::STOP);

        for (int i = 0; i < 13; i++)
        {
            this->setThresholds(i, _touch_threshold, _release_threshold);
        }

        write8((uint8_t)reg::MHDR, 0x01);
        write8((uint8_t)reg::NHDR, 0x01);
        // write8((uint8_t)reg::NCLR, 0x00);
        // write8((uint8_t)reg::FDLR, 0x00);
        write8((uint8_t)reg::MHDF, 0x01);
        write8((uint8_t)reg::NHDF, 0x01);
        write8((uint8_t)reg::NCLF, 0xFF);
        write8((uint8_t)reg::FDLF, 0x02);
        // write8((uint8_t)reg::NHDT, 0x00);
        // write8((uint8_t)reg::NCLT, 0x00);
        // write8((uint8_t)reg::FDLT, 0x00);
        // write8((uint8_t)reg::DEBOUNCE, 0x00);

        write8((uint8_t)reg::ECR, (uint8_t)command::START);

        return _sensor_started;
    };

    void setThresholds(uint8_t channel, uint8_t touch_threshold, uint8_t release_threshold)
    {
        uint8_t _channel = constrain(channel, 0, 12);
        write8((uint8_t)reg::TOUCHTH_0 + 2 * _channel, touch_threshold);
        write8((uint8_t)reg::RELEASETH_0 + 2 * _channel, release_threshold);
    };

    bool pressed()
    {
        if (_sensor_started)
        {
            this->readAll();
            return (_reading & 0x0FFF != 0);
        }
        return false;
    }

    uint8_t lastPressed()
    {
        if (_sensor_started)
        {
            this->readAll();
            return _last_pressed;
        }
        return 0;
    }

    uint8_t lastReleased()
    {
        if (_sensor_started)
        {
            this->readAll();
            return _last_released;
        }
        return 0;
    }

    bool read(uint8_t channel)
    {
        if (_sensor_started)
        {
            uint8_t channelc = constrain(channel, 0, 12);
            this->readAll();
            uint16_t mask = 0b1 << channelc;
            return (bool)((_reading & mask) >> channelc);
        }
        return 0;
    };

    evn_touch readAll()
    {
        evn_touch reading;

        if (_sensor_started)
        {
            uint16_t new_reading = read16((uint16_t)reg::TOUCH_STATUS_0_7, true, false) & 0x0FFF;

            for (int i = 0; i < 13; i++)
            {
                uint16_t mask = 0b1 << i;
                bool old_status = (bool)((_reading & mask) >> i);
                bool new_status = (bool)((new_reading & mask) >> i);

                switch (i)
                {
                case 0:
                    reading.ch0 = new_status;
                    break;
                case 1:
                    reading.ch1 = new_status;
                    break;
                case 2:
                    reading.ch2 = new_status;
                    break;
                case 3:
                    reading.ch3 = new_status;
                    break;
                case 4:
                    reading.ch4 = new_status;
                    break;
                case 5:
                    reading.ch5 = new_status;
                    break;
                case 6:
                    reading.ch6 = new_status;
                    break;
                case 7:
                    reading.ch7 = new_status;
                    break;
                case 8:
                    reading.ch8 = new_status;
                    break;
                case 9:
                    reading.ch9 = new_status;
                    break;
                case 10:
                    reading.ch10 = new_status;
                    break;
                case 11:
                    reading.ch11 = new_status;
                    break;
                case 12:
                    reading.ch12 = new_status;
                    break;
                }

                if (i != 12)
                {
                    if (old_status && !new_status)
                        _last_released = i;

                    if (!old_status && new_status)
                        _last_pressed = i;
                }
            }

            _reading = new_reading;
            return reading;
        }

        return reading;
    }

private:
    uint8_t _touch_threshold, _release_threshold;
    uint8_t _last_pressed = 0, _last_released = 0;
    uint16_t _reading = 0;
};

#endif