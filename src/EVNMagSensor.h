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
        setSamples((uint8_t)samples::SAMPLES_2);
        setRange((uint8_t)range::RANGE_2_5GA);
        setMeasurementMode((uint8_t)mode::MODE_CONTINUOUS);
        return true;
    };

    void setRange(uint8_t range) {
        write8(I2C_ADDR, reg::REG_CONFIG_B, range << 5);
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

    // calibrate in begin()
    bool isCalibrated()
    {
        bool output = _calibrated;
        return output;
    };

    void update()
    {
        _x = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_X_M);
        _y = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_Y_M);
        _z = read16(I2C_ADDR, (uint8_t)reg::REG_OUT_Z_M);
    };

    int16_t readRawX()
    {
        this->update();
        return _x;
    };

    int16_t readRawY()
    {
        this->update();
        return _y;
    };

    int16_t readRawZ()
    {
        this->update();
        return _z;
    };

    void calibrate()
    {
        double x0, y0, z0;
        x0 = (double)_x - _x_hard_offset;
        y0 = (double)_y - _y_hard_offset;
        z0 = (double)_z - _z_hard_offset;
        _xcal = x0 * _x_soft_offset[0] + y0 * _x_soft_offset[1] + z0 * _x_soft_offset[2];
        _ycal = x0 * _y_soft_offset[0] + y0 * _y_soft_offset[1] + z0 * _y_soft_offset[2];
        _zcal = x0 * _z_soft_offset[0] + y0 * _z_soft_offset[1] + z0 * _z_soft_offset[2];
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
    double _x_hard_offset, _x_soft_offset[3];
    double _y_hard_offset, _y_soft_offset[3];
    double _z_hard_offset, _z_soft_offset[3];
    uint32_t _last_reading_us;
    bool _calibrated = false;
};

#endif