#ifndef EVNCompassSensor_h
#define EVNCompassSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"
#include "EVNSensor.h"

// TODO: make settings viable without supplying register values

class EVNCompassSensor : private EVNSensor {
public:

    static const uint8_t HMC_I2C_ADDR = 0x1E;
    static const uint8_t HMC_CHIP_ID = 0x48 ^ 0x34 ^ 0x33;
    static const uint8_t QMC_I2C_ADDR = 0x0D;
    static const uint8_t QMC_CHIP_ID = 0xFF;

    enum hmc_reg
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

    enum class hmc_mode : uint8_t
    {
        MODE_CONTINUOUS = 0x00,
        MODE_SINGLE = 0x01,
        MODE_IDLE = 0x02,
    };

    enum class hmc_datarate : uint8_t
    {
        DATARATE_75HZ = 0x06,
        DATARATE_30HZ = 0x05,
        DATARATE_15HZ = 0x04,
        DATARATE_7_5HZ = 0x03,
        DATARATE_3HZ = 0x02,
        DATARATE_1_5HZ = 0x01,
        DATARATE_0_75HZ = 0x00,
    };

    enum class hmc_range : uint8_t
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

    enum class hmc_samples : uint8_t {
        SAMPLES_1 = 0x00,
        SAMPLES_2 = 0x01,
        SAMPLES_4 = 0x02,
        SAMPLES_8 = 0x03,
    };

    enum qmc_reg
    {
        REG_X_L = 0x00,
        REG_X_M = 0x01,
        REG_Y_L = 0x02,
        REG_Y_M = 0x03,
        REG_Z_L = 0x04,
        REG_Z_M = 0x05,
        REG_QMC_STATUS = 0x06,
        REG_TEMP_M = 0x07,
        REG_TEMP_L = 0x08,
        REG_CONFIG = 0x09,
        REG_CONFIG2 = 0x0A,
        REG_RESET = 0x0B,
        REG_RESERVED = 0x0C,
        REG_CHIP_ID = 0x0D,
    };

    enum class qmc_mode : uint8_t
    {
        MODE_CONTINUOUS = 0x01,
        MODE_STANDBY = 0x00,
    };

    enum class qmc_datarate : uint8_t
    {
        DATARATE_200HZ = 0x03,
        DATARATE_100HZ = 0x02,
        DATARATE_50HZ = 0x01,
        DATARATE_10HZ = 0x00,
    };

    enum class qmc_range : uint8_t
    {
        RANGE_8GA = 0x01,
        RANGE_2GA = 0x00,
    };

    enum class qmc_samples : uint8_t {
        SAMPLES_64 = 0x03,
        SAMPLES_128 = 0x02,
        SAMPLES_256 = 0x01,
        SAMPLES_512 = 0x00,
    };

    EVNCompassSensor(uint8_t port) : EVNSensor(port)
    {
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        uint8_t id =
            read8(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_IDEN_A) ^
            read8(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_IDEN_B) ^
            read8(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_IDEN_C);

        if (id != HMC_CHIP_ID)
        {
            id = read8(QMC_I2C_ADDR, (uint8_t)qmc_reg::REG_CHIP_ID);
            if (id == QMC_CHIP_ID)
            {
                _isqmc = true;
            }
            else
            {
                return false;
            }
        }

        if (_isqmc)
        {
            setMeasurementMode((uint8_t)qmc_mode::MODE_CONTINUOUS);
            setDataRate((uint8_t)qmc_datarate::DATARATE_50HZ);
            setRange((uint8_t)qmc_range::RANGE_8GA);
            setSamples((uint8_t)qmc_samples::SAMPLES_256);
        }
        else
        {
            setMeasurementMode((uint8_t)hmc_mode::MODE_CONTINUOUS);
            setDataRate((uint8_t)hmc_datarate::DATARATE_75HZ);
            setRange((uint8_t)hmc_range::RANGE_4GA);
            setSamples((uint8_t)hmc_samples::SAMPLES_1);
        }
        return true;
    };

    bool isQMC()
    {
        return _isqmc;
    };

    bool isHMC()
    {
        return !_isqmc;
    };

    void setMeasurementMode(uint8_t mode)
    {
        if (_isqmc)
        {
            uint8_t value;
            value = read8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG);
            value &= 0b11111100;
            value |= mode;
            write8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG, value);
        }
        else {
            uint8_t value;
            value = read8(HMC_I2C_ADDR, hmc_reg::REG_MODE);
            value &= 0b11111100;
            value |= mode;
            write8(HMC_I2C_ADDR, hmc_reg::REG_MODE, value);
        }
    };

    void setDataRate(uint8_t dataRate)
    {
        if (_isqmc)
        {
            switch ((qmc_datarate)dataRate)
            {
            case qmc_datarate::DATARATE_10HZ:
                _freq = 10;
                break;

            case qmc_datarate::DATARATE_50HZ:
                _freq = 50;
                break;

            case qmc_datarate::DATARATE_100HZ:
                _freq = 100;
                break;

            case qmc_datarate::DATARATE_200HZ:
                _freq = 200;
                break;
            }
            uint8_t value;
            value = read8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG);
            value &= 0b11110011;
            value |= (dataRate << 2);
            write8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG, value);
        }
        else {
            switch ((hmc_datarate)dataRate)
            {
            case hmc_datarate::DATARATE_0_75HZ:
                _freq = 0.75;
                break;
            case hmc_datarate::DATARATE_1_5HZ:
                _freq = 1.5;
                break;
            case hmc_datarate::DATARATE_3HZ:
                _freq = 3;
                break;
            case hmc_datarate::DATARATE_7_5HZ:
                _freq = 7.5;
                break;
            case hmc_datarate::DATARATE_15HZ:
                _freq = 15;
                break;
            case hmc_datarate::DATARATE_30HZ:
                _freq = 30;
                break;
            case hmc_datarate::DATARATE_75HZ:
                _freq = 75;
                break;
            }

            uint8_t value;
            value = read8(HMC_I2C_ADDR, hmc_reg::REG_CONFIG_A);
            value &= 0b11100011;
            value |= (dataRate << 2);
            write8(HMC_I2C_ADDR, hmc_reg::REG_CONFIG_A, value);
        }
    };

    void setRange(uint8_t range_value) {
        if (_isqmc)
        {
            switch ((qmc_range)range_value)
            {
            case qmc_range::RANGE_2GA:
                _gain = 12000;
                break;

            case qmc_range::RANGE_8GA:
                _gain = 3000;
                break;
            }
            uint8_t value;
            value = read8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG);
            value &= 0b11001111;
            value |= (range_value << 4);
            write8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG, value);
        }
        else {
            switch ((hmc_range)range_value)
            {
            case hmc_range::RANGE_0_88GA:
                _gain = 1370;
                break;
            case hmc_range::RANGE_1_3GA:
                _gain = 1090;
                break;
            case hmc_range::RANGE_1_9GA:
                _gain = 820;
                break;
            case hmc_range::RANGE_2_5GA:
                _gain = 660;
                break;
            case hmc_range::RANGE_4GA:
                _gain = 440;
                break;
            case hmc_range::RANGE_4_7GA:
                _gain = 390;
                break;
            case hmc_range::RANGE_5_6GA:
                _gain = 330;
                break;
            case hmc_range::RANGE_8_1GA:
                _gain = 230;
                break;
            }
            write8(HMC_I2C_ADDR, hmc_reg::REG_CONFIG_B, range_value << 5);
        }
    };

    void setSamples(uint8_t samples)
    {
        if (_isqmc)
        {
            uint8_t value;
            value = read8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG);
            value &= 0b00111111;
            value |= (samples << 6);
            write8(QMC_I2C_ADDR, qmc_reg::REG_CONFIG, value);
        }
        else {
            uint8_t value;
            value = read8(HMC_I2C_ADDR, hmc_reg::REG_CONFIG_A);
            value &= 0b10011111;
            value |= (samples << 5);
            write8(HMC_I2C_ADDR, hmc_reg::REG_CONFIG_A, value);
        }
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
            if (_isqmc)
            {
                _x = read16(QMC_I2C_ADDR, (uint8_t)qmc_reg::REG_X_L);
                _y = read16(QMC_I2C_ADDR, (uint8_t)qmc_reg::REG_Y_L);
                _z = read16(QMC_I2C_ADDR, (uint8_t)qmc_reg::REG_Z_L);

            }
            else {
                _x = read16(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_OUT_X_M, false);
                _y = read16(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_OUT_Y_M, false);
                _z = read16(HMC_I2C_ADDR, (uint8_t)hmc_reg::REG_OUT_Z_M, false);
            }
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
        x0 = (double)_x / _gain - _x_hard_cal;
        y0 = (double)_y / _gain - _y_hard_cal;
        z0 = (double)_z / _gain - _z_hard_cal;
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
            _xcal = (double)_x / _gain;
            _ycal = (double)_y / _gain;
            _zcal = (double)_z / _gain;
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
    bool _calibrated = false, _isqmc = false;
};

#endif