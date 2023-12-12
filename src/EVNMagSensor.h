#ifndef EVNMagSensor_h
#define EVNMagSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"
#include "EVNSensor.h"
#include "EEPROM.h"

class EVNMagSensor : private EVNSensor {
public:

    static const uint8_t I2C_ADDR = 0x1E;
    static const uint8_t ID_REG_PART_NUMBER = 0x48 ^ 0x34 ^ 0x33;

    enum reg
    {
        REG_CONFIG_A = 0x00,
        REG_CONFIG_B = 0x01,
        REG_MODE = 0x02,
        REG_OUT_X_M = 0x03,
        REG_OUT_X_L = 0x04,
        REG_OUT_Z_M = 0x05,
        REG_OUT_Z_L = 0x06,
        REG_OUT_Y_M = 0x07,
        REG_OUT_Y_L = 0x08,
        REG_STATUS = 0x09,
        REG_IDEN_A = 0x0A,
        REG_IDEN_B = 0x0B,
        REG_IDEN_C = 0x0C,
    };

    enum class mode : uint8_t
    {
        MODE_CONTINUOUS = 0x00,
        MODE_SINGLE = 0x01,
        MODE_IDLE = 0x02,
    };

    enum class datarate : uint8_t
    {
        DATARATE_75HZ = 0x06,
        DATARATE_30HZ = 0x05,
        DATARATE_15HZ = 0x04,
        DATARATE_7_5HZ = 0x03,
        DATARATE_3HZ = 0x02,
        DATARATE_1_5HZ = 0x01,
        DATARATE_0_75HZ = 0x00,
    };

    enum class range : uint8_t
    {
        RANGE_8_1GA = 0x07,
        RANGE_5_6GA = 0x06,
        RANGE_4_7GA = 0x05,
        RANGE_4GA = 0x04,
        RANGE_2_5GA = 0x03,
        RANGE_1_9GA = 0x02,
        RANGE_1_3GA = 0x01,
        RANGE_0_88GA = 0x00,
    };

    enum class samples : uint8_t {
        SAMPLES_1 = 0x00,
        SAMPLES_2 = 0x01,
        SAMPLES_4 = 0x02,
        SAMPLES_8 = 0x03,
    };

    EVNMagSensor(uint8_t port) : EVNSensor(port)
    {
    };

    bool begin()
    {
        this->beginPortSelector();

        uint8_t id =
            read8(I2C_ADDR, (uint8_t)reg::REG_IDEN_A) ^
            read8(I2C_ADDR, (uint8_t)reg::REG_IDEN_B) ^
            read8(I2C_ADDR, (uint8_t)reg::REG_IDEN_C);

        if (id != ID_REG_PART_NUMBER) return false;

        setDataRate((uint8_t)datarate::DATARATE_75HZ);
        setSamples((uint8_t)samples::SAMPLES_1);
        setRange((uint8_t)range::RANGE_4GA);
        setMeasurementMode((uint8_t)mode::MODE_CONTINUOUS);

        // write8(I2C_ADDR, (uint8_t)reg::REG_CONFIG_A, 0x70);
        // write8(I2C_ADDR, (uint8_t)reg::REG_CONFIG_B, 0xA0);
        // write8(I2C_ADDR, (uint8_t)reg::REG_MODE, 0x00);
        // _freq = 15;
        // _gain = 390;
        return true;
    };

    void setRange(uint8_t value) {
        switch ((range)value)
        {
        case range::RANGE_0_88GA:
            _gain = 1370;
            break;
        case range::RANGE_1_3GA:
            _gain = 1090;
            break;
        case range::RANGE_1_9GA:
            _gain = 820;
            break;
        case range::RANGE_2_5GA:
            _gain = 660;
            break;
        case range::RANGE_4GA:
            _gain = 440;
            break;
        case range::RANGE_4_7GA:
            _gain = 390;
            break;
        case range::RANGE_5_6GA:
            _gain = 330;
            break;
        case range::RANGE_8_1GA:
            _gain = 230;
            break;
        }
        write8(I2C_ADDR, reg::REG_CONFIG_B, value << 5);
    };

    void setMeasurementMode(uint8_t mode)
    {
        uint8_t value;
        value = read8(I2C_ADDR, reg::REG_MODE);
        value &= 0b11111100;
        value |= mode;
        write8(I2C_ADDR, reg::REG_MODE, value);
    };

    void setDataRate(uint8_t dataRate)
    {
        switch ((datarate)dataRate)
        {
        case datarate::DATARATE_0_75HZ:
            _freq = 0.75;
            break;
        case datarate::DATARATE_1_5HZ:
            _freq = 1.5;
            break;
        case datarate::DATARATE_3HZ:
            _freq = 3;
            break;
        case datarate::DATARATE_7_5HZ:
            _freq = 7.5;
            break;
        case datarate::DATARATE_15HZ:
            _freq = 15;
            break;
        case datarate::DATARATE_30HZ:
            _freq = 30;
            break;
        case datarate::DATARATE_75HZ:
            _freq = 75;
            break;
        }

        uint8_t value;
        value = read8(I2C_ADDR, reg::REG_CONFIG_A);
        value &= 0b11100011;
        value |= (dataRate << 2);
        write8(I2C_ADDR, reg::REG_CONFIG_A, value);
    };

    void setSamples(uint8_t samples)
    {
        uint8_t value;
        value = read8(I2C_ADDR, reg::REG_CONFIG_A);
        value &= 0b10011111;
        value |= (samples << 5);
        write8(I2C_ADDR, reg::REG_CONFIG_A, value);
    };

    bool isCalibrated()
    {
        bool output = _calibrated;
        return output;
    };

    void update()
    {
        if ((micros() - _last_reading_us) >= (1000000 / _freq))
        {
            _x = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_X_M, false);
            _y = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_Y_M, false);
            _z = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_Z_M, false);
            _last_reading_us = micros();
        }
    };

    double readRawX()
    {
        this->update();
        return (double)_x / _gain;
    };

    double readRawY()
    {
        this->update();
        return (double)_y / _gain;
    };

    double readRawZ()
    {
        this->update();
        return (double)_z / _gain;
    };

    void calibrate()
    {
        double x0, y0, z0;
        x0 = (double)_x - _x_hard_cal;
        y0 = (double)_y - _y_hard_cal;
        z0 = (double)_z - _z_hard_cal;
        _xcal = x0 * _x_soft_cal[0] + y0 * _x_soft_cal[1] + z0 * _x_soft_cal[2];
        _ycal = x0 * _y_soft_cal[0] + y0 * _y_soft_cal[1] + z0 * _y_soft_cal[2];
        _zcal = x0 * _z_soft_cal[0] + y0 * _z_soft_cal[1] + z0 * _z_soft_cal[2];
    };

    double read() {
        this->update();
        if (_calibrated) {
            this->calibrate();
        }
        else {
            _xcal = (double)_x;
            _ycal = (double)_y;
            _zcal = (double)_z;
        }

        _reading = (atan2(_ycal, _xcal) / M_PI * 180) + 180;
        return fmod(_reading - _yaw_offset + 360, 360);
    };

    void setNorth()
    {
        _yaw_offset = this->read();
    };

private:
    int16_t _reading, _x, _y, _z;
    double _xcal, _ycal, _zcal;
    double _yaw_offset = 0;
    double _x_hard_cal = 0, _x_soft_cal[3] = { 1,0,0 };
    double _y_hard_cal = 0, _y_soft_cal[3] = { 0,1,0 };
    double _z_hard_cal = 0, _z_soft_cal[3] = { 0,0,1 };
    double _freq, _gain;
    uint32_t _last_reading_us = 0;
    bool _calibrated = false;
};

#endif