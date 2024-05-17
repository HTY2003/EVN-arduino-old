#ifndef EVNIMUSensor_h
#define EVNIMUSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

class EVNIMUSensor : private EVNI2CDevice {
public:
    static const uint8_t I2C_ADDR = 0x68;
    static const uint8_t ID = 0x70;

    enum class reg : uint8_t
    {
        SMPLRT_DIV = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        ACCEL_CONFIG2 = 0x1D,
        LP_ACCEL_ODR = 0x1E,
        WOM_THR = 0x1F,
        FIFO_EN = 0x23,
        //I2C CONTROL REGS SKIPPED
        INT_PIN_CFG = 0x37,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3A,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        //I2C CONTROL REGS SKIPPED
        SIGNAL_PATH_RESET = 0x68,
        ACCEL_INTEL_CTRL = 0x69,
        USER_CTRL = 0x6A,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        FIFO_COUNT_H = 0x72,
        FIFO_COUNT_L = 0x73,
        FIFO_R_W = 0x74,
        WHO_AM_I = 0x75,
        XA_OFFSET_H = 0x77,
        XA_OFFSET_L = 0x78,
        YA_OFFSET_H = 0x7A,
        YA_OFFSET_L = 0x7B,
        ZA_OFFSET_H = 0x7D,
        ZA_OFFSET_L = 0x7E
    };

    enum class gyro_range : uint8_t
    {
        DPS_250 = 0,
        DPS_500 = 1,
        DPS_1000 = 2,
        DPS_2000 = 3
    };

    enum class accel_range : uint8_t
    {
        G_2 = 0,
        G_4 = 1,
        G_8 = 2,
        G_16 = 3
    };

    enum class data_rate : uint8_t
    {
        // HZ_460 = 0, - accel
        // HZ_250 = 0x00, - gyro
        HZ_184 = 1,
        HZ_92 = 2,
        HZ_41 = 3,
        HZ_20 = 4,
        HZ_10 = 5,
        HZ_5 = 6
    };

    //TODO: Maybe explore low power mode and sample rate divider in future

    EVNIMUSensor(uint8_t port) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        write8((uint8_t)reg::PWR_MGMT_1, 0x80);
        delay(10);
        uint8_t id = read8((uint8_t)reg::WHO_AM_I, false);

        if (id != ID)
        {
            return _sensor_started;
        }

        _sensor_started = true;

        setAccelRange(accel_range::G_4);
        setGyroRange(gyro_range::DPS_500);
        setDataRate(data_rate::HZ_92);

        return _sensor_started;
    };

    void setAccelRange(accel_range range)
    {
        switch (range)
        {
        case accel_range::G_2:
            _accel_sens = 16384;
            break;
        case accel_range::G_4:
            _accel_sens = 8192;
            break;
        case accel_range::G_8:
            _accel_sens = 4096;
            break;
        case accel_range::G_16:
            _accel_sens = 2048;
            break;
        }

        uint8_t value = read8((uint8_t)reg::ACCEL_CONFIG);
        value &= 0b11100111;
        value |= ((uint8_t)range << 3);
        write8((uint8_t)reg::ACCEL_CONFIG, value);
    };

    void setGyroRange(gyro_range range)
    {
        switch (range)
        {
        case gyro_range::DPS_250:
            _gyro_sens = 131;
            break;
        case gyro_range::DPS_500:
            _gyro_sens = 65.5;
            break;
        case gyro_range::DPS_1000:
            _gyro_sens = 32.8;
            break;
        case gyro_range::DPS_2000:
            _gyro_sens = 16.4;
            break;
        }

        uint8_t value = read8((uint8_t)reg::GYRO_CONFIG);
        value &= 0b11100111;
        value |= ((uint8_t)range << 3);
        write8((uint8_t)reg::GYRO_CONFIG, value);
    };

    void setDataRate(data_rate data_rate)
    {
        switch (data_rate)
        {
        case data_rate::HZ_5:
            _measurement_time_us = 200000;
            break;
        case data_rate::HZ_10:
            _measurement_time_us = 100000;
            break;
        case data_rate::HZ_20:
            _measurement_time_us = 50000;
            break;
        case data_rate::HZ_41:
            _measurement_time_us = 24391;
            break;
        case data_rate::HZ_92:
            _measurement_time_us = 10870;
            break;
        case data_rate::HZ_184:
            _measurement_time_us = 5435;
            break;
        }

        uint8_t value = read8((uint8_t)reg::GYRO_CONFIG);
        value &= 0b11111100;
        write8((uint8_t)reg::GYRO_CONFIG, value);

        value = read8((uint8_t)reg::CONFIG);
        value &= 0b11111000;
        value |= (uint8_t)data_rate;
        write8((uint8_t)reg::CONFIG, value);

        value = read8((uint8_t)reg::ACCEL_CONFIG2);
        value &= 0b11110000;
        value |= ((uint8_t)data_rate);
        write8((uint8_t)reg::ACCEL_CONFIG2, value);
    };

    float readAccelX(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _ax;
        }
        return 0;
    };

    float readAccelY(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _ay;
        }
        return 0;
    };

    float readAccelZ(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _az;
        }
        return 0;
    };

    float readGyroX(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _gx;
        }
        return 0;
    };

    float readGyroY(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _gy;
        }
        return 0;
    };

    float readGyroZ(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _gz;
        }
        return 0;
    };

private:
    void update(bool blocking = false)
    {
        if (blocking)
            while (micros() - _last_reading_us < _measurement_time_us);

        if (micros() - _last_reading_us >= _measurement_time_us)
        {
            readBuffer((uint8_t)reg::ACCEL_XOUT_H, 14, _buffer, false);
            _last_reading_us = micros();

            _axr = _buffer[0] << 8 | _buffer[1];
            _ayr = _buffer[2] << 8 | _buffer[3];
            _azr = _buffer[4] << 8 | _buffer[5];
            _temp = _buffer[6] << 8 | _buffer[7];
            _gxr = _buffer[8] << 8 | _buffer[9];
            _gyr = _buffer[10] << 8 | _buffer[11];
            _gzr = _buffer[12] << 8 | _buffer[13];

            _ax = (float)_axr / _accel_sens;
            _ay = (float)_ayr / _accel_sens;
            _az = (float)_azr / _accel_sens;

            _gx = (float)_gxr / _gyro_sens;
            _gy = (float)_gyr / _gyro_sens;
            _gz = (float)_gzr / _gyro_sens;
        }
    };

    uint8_t _buffer[14];
    int16_t _temp;
    int16_t _axr = 0, _ayr = 0, _azr = 0;
    int16_t _gxr = 0, _gyr = 0, _gzr = 0;
    float _ax = 0, _ay = 0, _az = 0;
    float _gx = 0, _gy = 0, _gz = 0;
    float _gyro_sens;
    float _accel_sens;
    uint64_t _measurement_time_us;
    uint64_t _last_reading_us;
};

#endif