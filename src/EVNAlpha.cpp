#include "EVNAlpha.h"

EVNButton EVNAlpha::button(_mode, _linkLED, _linkMotors);
EVNPortSelector EVNAlpha::ports(_i2c_freq);
uint8_t EVNAlpha::_mode;
uint8_t EVNAlpha::_linkLED;
uint8_t EVNAlpha::_linkMotors;
uint32_t EVNAlpha::_i2c_freq;
// bool EVNAlpha::_has_battery_adc;

EVNAlpha::EVNAlpha(uint8_t mode, uint8_t linkLED, uint8_t linkMotors, uint32_t i2c_freq)
{
    _mode = constrain(mode, 0, 2);
    _linkLED = constrain(linkLED, 0, 1);
    _linkMotors = constrain(linkMotors, 0, 1);
    _i2c_freq = i2c_freq;
}

void EVNAlpha::begin()
{
    //Set correct I2C and Serial pins
    //TODO: To be removed after getting our board in arduino-pico core
    Wire.setSDA(WIRE0_SDA);
    Wire.setSCL(WIRE0_SCL);
    Wire1.setSDA(WIRE1_SDA);
    Wire1.setSCL(WIRE1_SCL);

    Serial1.setRX(SERIAL1_RX);
    Serial1.setTX(SERIAL1_TX);
    Serial2.setRX(SERIAL2_RX);
    Serial2.setTX(SERIAL2_TX);

    //Initialize helper objects
    ports.begin();
    button.begin();

    // if (batt.begin()) _has_battery_adc = true;
}